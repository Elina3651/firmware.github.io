//==========================================================================
// Библиотеки
//==========================================================================
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <LEDStripDriver.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_MultiGasSensor.h"

//==========================================================================
// Конфигурация пинов и устройств
//==========================================================================

// --- UART для связи с главной платой ---
#define MAIN_BOARD_RX_PIN 17
#define MAIN_BOARD_TX_PIN 16
HardwareSerial SerialPort(2);

// --- Драйверы LED Strip (управляют клапанами/вентиляторами/мотором) ---
const int DRIVER1_DIN_PIN = 12;
const int DRIVER1_CIN_PIN = 13;
const int DRIVER2_DIN_PIN = 27;
const int DRIVER2_CIN_PIN = 14;
LEDStripDriver driver1(DRIVER1_DIN_PIN, DRIVER1_CIN_PIN);
LEDStripDriver driver2(DRIVER2_DIN_PIN, DRIVER2_CIN_PIN);

// --- Пины MOSFET (управляют нагревателем и УФ-линиями) ---
const int HEATER_PIN = 26;      // MOSFET 1
const int UV_380_PIN = 25;      // MOSFET 2
const int UV_430_PIN = 33;      // MOSFET 3

// --- Датчики ---
const int DOOR_SENSOR_PIN = 15;
const int ONE_WIRE_BUS_PIN = 23;
const int O2_SDA_PIN = 21;
const int O2_SCL_PIN = 22;

// --- Объекты датчиков ---
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature tempSensor(&oneWire);
DFRobot_GAS_I2C gasSensor(&Wire, 0x74);
bool ds18b20_found = false;
bool o2_sensor_found = false;

//==========================================================================
// Машина состояний и структура данных процесса
//==========================================================================

// --- Состояния (этапы) процесса ---
enum ProcessState {
    IDLE,
    PREPARATION,
    HEATING_HOLDING,
    NITROGEN_PURGE,
    PRIMARY_UV,
    SECONDARY_UV,
    TERTIARY_UV,
    POST_COOLING
};
ProcessState currentState = IDLE;

enum HeatingSubState {
    HS_IDLE,
    HS_HEATING,
    HS_COOLING,
    HS_HOLDING
};
HeatingSubState heatingState = HS_IDLE;

// --- Структура для хранения параметров текущего профиля ---
struct ProcessParameters {
    bool thermal_chamber_enabled;
    int thermal_chamber_temp;
    int heat_exchange_hold_sec;
    bool nitrogen_use_enabled;
    int nitrogen_target_percent;
    int primary_uv_exposure_sec;
    int primary_uv_mode;
    int primary_uv_flicker_rate;
    int secondary_uv_exposure_sec;
    int secondary_uv_mode;
    int tertiary_uv_exposure_sec;
    int tertiary_uv_mode;
    bool chamber_cooling_enabled;
};
ProcessParameters currentProcess;

// --- Переменные для таймеров и проверок ---
unsigned long state_timer_start = 0;
unsigned long error_check_timer_start = 0;
float value_at_error_check_start = 0.0f;
unsigned long last_telemetry_send_time = 0;

// --- Виртуальное состояние драйверов ---
uint8_t driver1_r = 0, driver1_g = 0, driver1_b = 0;
uint8_t driver2_r = 0, driver2_g = 0, driver2_b = 0;

//==========================================================================
// Вспомогательные функции управления периферией
//==========================================================================

/** @brief Применяет виртуальное состояние (глобальные переменные) к физическим драйверам. */
void applyDriverState() {
    driver1.setColor(driver1_r, driver1_g, driver1_b);
    driver2.setColor(driver2_r, driver2_g, driver2_b);
}

/** @brief Выключает АБСОЛЮТНО ВСЮ силовую периферию. Функция безопасности. */
void all_off() {
    Serial.println("!!! EMERGENCY/ALL OFF triggered !!!");
    // Сбрасываем виртуальное состояние
    driver1_r = driver1_g = driver1_b = 0;
    driver2_r = driver2_g = driver2_b = 0;
    // Применяем его
    applyDriverState();
    // Управляем MOSFET'ами напрямую
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(UV_380_PIN, LOW);
    digitalWrite(UV_430_PIN, LOW);
}

// --- Функции для опроса датчиков ---
float readTemperature() {
    if (!ds18b20_found) return -127.0;
    tempSensor.requestTemperatures();
    return tempSensor.getTempCByIndex(0);
}

/**
 * @brief Читает концентрацию кислорода или эмулирует ее, если датчик не найден.
 * @return Концентрация O2 в процентах (%).
 */
float readOxygenLevel() {
    // --- РЕЖИМ ЭМУЛЯЦИИ ---
    if (!o2_sensor_found) {
        // Если датчик физически не найден при старте,
        // возвращаем стандартное значение атмосферного кислорода.
        return 20.8f; 
    }
    
    // --- РЕЖИМ РЕАЛЬНОЙ РАБОТЫ ---
    float o2_concentration = gasSensor.readGasConcentrationPPM();
    
    // Если датчик есть, но вернул ошибку (например, < 0), 
    // чтобы не ломать логику, тоже вернем стандартное значение.
    if (o2_concentration < 0) { 
        Serial.printf("Ошибка чтения с датчика O2 (код: %.1f), возвращаем стандартное значение.\n", o2_concentration);
        return 20.8f;
    }
    
    return o2_concentration;
}

bool isDoorClosed() {
    return digitalRead(DOOR_SENSOR_PIN) == LOW;
}

//==========================================================================
// Логика машины состояний
//==========================================================================

void enterState(ProcessState newState); // Объявляем заранее

/** @brief Собирает и отправляет пакет с текущими данными (телеметрией) на главный экран. */
void sendTelemetry() {
    StaticJsonDocument<256> doc;
    char buffer[256];

    // Команда-идентификатор пакета
    doc["type"] = "TELEMETRY";

    // Данные датчиков
    doc["temp"] = readTemperature();
    doc["o2"] = readOxygenLevel();

    // Данные по таймерам этапов (в секундах)
    unsigned long elapsed_ms = millis() - state_timer_start;
    int remaining_sec = 0;

    switch(currentState) {
        // --- НАЧАЛО ИЗМЕНЕНИЙ ---
        case HEATING_HOLDING:
            // Таймер показываем, только если мы в под-состоянии удержания
            if (heatingState == HS_HOLDING) {
                remaining_sec = currentProcess.heat_exchange_hold_sec - (elapsed_ms / 1000);
                doc["timer_rem"] = max(0, remaining_sec);
            } else {
                doc["timer_rem"] = -1; // На этапах нагрева/охлаждения таймер не показываем
            }
            break;
        // --- КОНЕЦ ИЗМЕНЕНИЙ ---

        case PRIMARY_UV:
            remaining_sec = currentProcess.primary_uv_exposure_sec - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case SECONDARY_UV:
            remaining_sec = currentProcess.secondary_uv_exposure_sec - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case TERTIARY_UV:
            remaining_sec = currentProcess.tertiary_uv_exposure_sec - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case POST_COOLING:
            remaining_sec = 60 - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        default:
            doc["timer_rem"] = -1; // -1 означает, что таймер неактивен
            break;
    }

    serializeJson(doc, buffer);
    SerialPort.println(buffer);
}

/** @brief Обработчик команд, полученных по UART */
void handleCommand(const String& cmd) {
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, cmd);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }

    const char* command = doc["command"];

    if (strcmp(command, "START_PROCESS") == 0) {
        if (currentState != IDLE) {
            Serial.println("Warning: Received START_PROCESS while another process is running. Ignoring.");
            return;
        }
        
        // Парсим все параметры профиля
        JsonObject params = doc["params"];
        currentProcess.thermal_chamber_enabled = params["thermal_chamber_enabled"];
        currentProcess.thermal_chamber_temp = params["thermal_chamber_temp"];
        currentProcess.heat_exchange_hold_sec = params["heat_exchange_hold_sec"];
        currentProcess.nitrogen_use_enabled = params["nitrogen_use_enabled"];
        currentProcess.nitrogen_target_percent = params["nitrogen_target_percent"];
        currentProcess.primary_uv_exposure_sec = params["primary_uv_exposure_sec"];
        currentProcess.primary_uv_mode = params["primary_uv_mode"];
        currentProcess.primary_uv_flicker_rate = params["primary_uv_flicker_rate"];
        currentProcess.secondary_uv_exposure_sec = params["secondary_uv_exposure_sec"];
        currentProcess.secondary_uv_mode = params["secondary_uv_mode"];
        currentProcess.tertiary_uv_exposure_sec = params["tertiary_uv_exposure_sec"];
        currentProcess.tertiary_uv_mode = params["tertiary_uv_mode"];
        currentProcess.chamber_cooling_enabled = params["chamber_cooling_enabled"];
        
        Serial.println("Received START_PROCESS command. Parsed params. Entering PREPARATION state.");
        SerialPort.println("ACK:PROCESS_STARTED");
        enterState(PREPARATION);

    } else if (strcmp(command, "EMERGENCY_STOP") == 0) {
        Serial.println("Received EMERGENCY_STOP from main board.");
        all_off();
        SerialPort.println("ACK:STOP_COMMAND_RECEIVED");
        enterState(IDLE);
    }
}

/** @brief Функция смены состояния. Здесь описывается, что делать при ВХОДЕ в новое состояние. */
void enterState(ProcessState newState) {
    currentState = newState;
    state_timer_start = millis(); // Сбрасываем таймер для нового этапа

    switch (currentState) {
        case IDLE:
            all_off();
            Serial.println("Entering IDLE state.");
            break;

        case PREPARATION:
            Serial.println("Entering PREPARATION state.");
            if (!isDoorClosed()) {
                Serial.println("Error: Door is open. Aborting process.");
                SerialPort.println("ERROR:DOOR_IS_OPEN");
                enterState(IDLE);
                return;
            }
            // Включаем вентиляторы и мотор
            // Включаем Вентиляторы (Драйвер 1, канал R)
            driver1_r = 255; 
            // Включаем Мотор (Драйвер 2, канал R)
            driver2_r = 255;
            applyDriverState(); // Применяем изменения
            SerialPort.println("STATUS:PREPARATION_OK");
            enterState(HEATING_HOLDING); // Сразу переходим к следующему этапу
            break;

        case HEATING_HOLDING: {
            Serial.println("Entering HEATING_HOLDING state.");
            if (!currentProcess.thermal_chamber_enabled) {
                Serial.println("Thermal chamber disabled. Skipping.");
                heatingState = HS_IDLE; // Сбрасываем под-состояние
                enterState(NITROGEN_PURGE);
                return;
            }
            
            // Определяем начальное под-состояние
            float currentTemp = readTemperature();
            float targetTemp = currentProcess.thermal_chamber_temp;

            if (currentTemp < targetTemp - 0.5) { // Греем, если температура ниже цели (с гистерезисом)
                heatingState = HS_HEATING;
                driver1_b = 255; 
                applyDriverState();
                digitalWrite(HEATER_PIN, HIGH);
                error_check_timer_start = millis();
                value_at_error_check_start = currentTemp;
                SerialPort.println("STATUS:HEATING_STARTED");

            } else if (currentTemp > targetTemp + 0.5 && currentProcess.chamber_cooling_enabled) { // Охлаждаем, если выше и разрешено
                heatingState = HS_COOLING;
                driver1_g = 255;
                applyDriverState();
                error_check_timer_start = millis();
                value_at_error_check_start = currentTemp;
                SerialPort.println("STATUS:COOLING_STARTED");

            } else { // Если температура уже в норме или охлаждение запрещено
                heatingState = HS_HOLDING;
                state_timer_start = millis(); // Сразу начинаем удержание
                SerialPort.println("STATUS:HOLDING_TEMPERATURE");
            }
            break;
          }
        case NITROGEN_PURGE:
             Serial.println("Entering NITROGEN_PURGE state.");
            if (!currentProcess.nitrogen_use_enabled) {
                Serial.println("Nitrogen purge disabled. Skipping.");
                enterState(PRIMARY_UV);
                return;
            }
            // Включаем клапан азота
            driver2_g = 255;
            applyDriverState();
            error_check_timer_start = millis();
            value_at_error_check_start = readOxygenLevel();
            SerialPort.println("STATUS:NITROGEN_PURGE_STARTED");
            break;

        case PRIMARY_UV:
            Serial.println("Entering PRIMARY_UV state.");
            SerialPort.println("STATUS:PRIMARY_UV_STARTED");
            break;

        case SECONDARY_UV:
            Serial.println("Entering SECONDARY_UV state.");
            if (currentProcess.secondary_uv_mode == 0 || currentProcess.secondary_uv_mode == 2) digitalWrite(UV_380_PIN, HIGH);
            if (currentProcess.secondary_uv_mode == 1 || currentProcess.secondary_uv_mode == 2) digitalWrite(UV_430_PIN, HIGH);
            SerialPort.println("STATUS:SECONDARY_UV_STARTED");
            break;

        case TERTIARY_UV:
            Serial.println("Entering TERTIARY_UV state.");
            if (currentProcess.tertiary_uv_mode == 0 || currentProcess.tertiary_uv_mode == 2) digitalWrite(UV_380_PIN, HIGH);
            if (currentProcess.tertiary_uv_mode == 1 || currentProcess.tertiary_uv_mode == 2) digitalWrite(UV_430_PIN, HIGH);
            SerialPort.println("STATUS:TERTIARY_UV_STARTED"); // Можно добавить новый статус или использовать старый
            break;
            
        case POST_COOLING:
            Serial.println("Entering POST_COOLING state.");
            // Выключаем всё на втором драйвере (Мотор, Клапан N2)
            driver2_r = 0;
            driver2_g = 0;
            // На первом драйвере: Вентиляторы (R) остаются, включаем Вентилятор дуйки (B)
            driver1_b = 255;
            applyDriverState();
            SerialPort.println("STATUS:POST_COOLING_STARTED");
            break;
    }
}

/** @brief Основная логика, выполняющаяся в каждом цикле в зависимости от текущего состояния */
void state_machine_loop() {

    // --- Отправка телеметрии по таймеру ---
    if (currentState != IDLE && millis() - last_telemetry_send_time > 500) { // Отправляем каждые 500 мс
        sendTelemetry();
        last_telemetry_send_time = millis();
    }

    // Глобальная проверка безопасности: Дверь
    if (currentState != IDLE && currentState != POST_COOLING) {
        if (!isDoorClosed()) {
            Serial.println("!!! DOOR OPENED DURING PROCESS !!! EMERGENCY STOP !!!");
            all_off();
            SerialPort.println("EVENT:DOOR_OPENED_EMERGENCY_STOP");
            enterState(IDLE);
            return;
        }
    }

    switch (currentState) {
        case IDLE:
            // Ничего не делаем, просто ждем команду
            break;

        case HEATING_HOLDING: {
            if (heatingState == HS_IDLE) break; // Если мы в этом состоянии, но под-состояние не задано, ничего не делаем

            float currentTemp = readTemperature();
            float targetTemp = currentProcess.thermal_chamber_temp;

            switch (heatingState) {
                case HS_HEATING:
                    // Проверяем, достигли ли цели
                    if (currentTemp >= targetTemp) {
                        Serial.println("Target temperature reached. Starting hold timer.");
                        digitalWrite(HEATER_PIN, LOW);
                        driver1_b = 0;
                        applyDriverState();
                        state_timer_start = millis(); // Запускаем таймер удержания
                        heatingState = HS_HOLDING;
                        SerialPort.println("STATUS:HOLDING_TEMPERATURE");
                        break;
                    }

                    // Проверка на ошибку нагревателя
                    if (millis() - error_check_timer_start > 100000) { // 100 секунд
                        if (currentTemp - value_at_error_check_start < 1.0f) {
                            Serial.println("!!! FATAL ERROR: HEATER FAILURE !!!");
                            all_off();
                            SerialPort.println("FATAL_ERROR:HEATER_FAILURE");
                            enterState(IDLE); // Переходим в IDLE, но можно и в спец. состояние ошибки
                        } else {
                            error_check_timer_start = millis();
                            value_at_error_check_start = currentTemp;
                        }
                    }
                    break;

                case HS_COOLING:
                    // Проверяем, достигли ли цели
                    if (currentTemp <= targetTemp) {
                        Serial.println("Target temperature reached. Starting hold timer.");
                        driver1_g = 0;
                        applyDriverState();
                        state_timer_start = millis();
                        heatingState = HS_HOLDING;
                        SerialPort.println("STATUS:HOLDING_TEMPERATURE");
                        break;
                    }

                    // Проверка на ошибку охлаждения
                    if (millis() - error_check_timer_start > 100000) { // 100 секунд
                        if (value_at_error_check_start - currentTemp < 1.0f) {
                            Serial.println("Warning: Cooling timed out. Skipping.");
                            driver1_g = 0;
                            applyDriverState();
                            heatingState = HS_IDLE;
                            SerialPort.println("WARN:COOLING_SKIPPED");
                            enterState(NITROGEN_PURGE); // Пропускаем весь этап
                        } else {
                            error_check_timer_start = millis();
                            value_at_error_check_start = currentTemp;
                        }
                    }
                    break;

                case HS_HOLDING:
                    // "Климат-контроль" - если температура упала, немного подогреваем
                    if (currentTemp < targetTemp - 0.5) { // Гистерезис 0.5 градуса
                        digitalWrite(HEATER_PIN, HIGH);
                        // Также включаем вентилятор дуйки
                        driver1_b = 255; // Вкл. вентилятор дуйки
                        applyDriverState();
                    } else {
                        digitalWrite(HEATER_PIN, LOW);
                        // Выключаем вентилятор дуйки, если он был включен
                        driver1_b = 0; // Выкл. вентилятор дуйки
                        applyDriverState();
                    }

                    // Проверяем, закончилось ли время удержания
                    if (millis() - state_timer_start >= (unsigned long)currentProcess.heat_exchange_hold_sec * 1000) {
                        Serial.println("Hold time finished.");
                        all_off(); // Выключим всё перед переходом, чтобы гарантировать чистое состояние
                        driver1_r = 255; // Вентиляторы
                        driver2_r = 255; // Мотор
                        applyDriverState();
                        heatingState = HS_IDLE;
                        enterState(NITROGEN_PURGE);
                    }
                    break;
            }
            break;
        }

        case NITROGEN_PURGE: {
            float currentO2 = readOxygenLevel();
            float targetO2 = 100.0f - currentProcess.nitrogen_target_percent;
            if (currentO2 <= targetO2 && currentO2 > 0) {
                Serial.printf("Nitrogen purge complete (O2 <= %.1f%%).\n", targetO2);
                // Выключаем клапан азота (Драйвер 2, канал G)
                driver2_g = 0;
                applyDriverState();
                enterState(PRIMARY_UV);
                return;
            }
            // Проверка на ошибку (закончился азот)
            if (millis() - error_check_timer_start > 100000) { // 100 секунд
                if (value_at_error_check_start - currentO2 < 1.0f) {
                    Serial.println("Warning: Nitrogen purge timed out. Skipping.");
                    // Выключаем клапан
                    driver2_g = 0;
                    applyDriverState();
                    SerialPort.println("WARN:NITROGEN_SKIPPED");
                    enterState(PRIMARY_UV);
                } else { // Сбрасываем таймер, если прогресс есть
                    error_check_timer_start = millis();
                    value_at_error_check_start = currentO2;
                }
            }
            break;
        }

        case PRIMARY_UV: {
            unsigned long elapsed = millis() - state_timer_start;
            if (elapsed >= (unsigned long)currentProcess.primary_uv_exposure_sec * 1000) {
                digitalWrite(UV_380_PIN, LOW);
                digitalWrite(UV_430_PIN, LOW);
                Serial.println("Primary UV finished.");
                enterState(SECONDARY_UV);
                return;
            }

            // Логика мерцания
            bool is_flickering_on = false;
            if (currentProcess.primary_uv_flicker_rate > 0) {
                float period_ms = 1000.0f / currentProcess.primary_uv_flicker_rate;
                is_flickering_on = (fmod(elapsed, period_ms) < period_ms / 2.0f); // Скважность 50%
            }

            // Включаем нужные диоды, если сейчас фаза "ВКЛ"
            if (is_flickering_on) {
                if (currentProcess.primary_uv_mode == 0 || currentProcess.primary_uv_mode == 2) { // Type 1 или Both
                    digitalWrite(UV_380_PIN, HIGH);
                }
                if (currentProcess.primary_uv_mode == 1 || currentProcess.primary_uv_mode == 2) { // Type 2 или Both
                    digitalWrite(UV_430_PIN, HIGH);
                }
            } else {
                // Выключаем все, если сейчас фаза "ВЫКЛ"
                digitalWrite(UV_380_PIN, LOW);
                digitalWrite(UV_430_PIN, LOW);
            }
            break;
        }

        case SECONDARY_UV:
            if (millis() - state_timer_start >= (unsigned long)currentProcess.secondary_uv_exposure_sec * 1000) {
                digitalWrite(UV_380_PIN, LOW);
                digitalWrite(UV_430_PIN, LOW);
                Serial.println("Secondary UV finished.");
                enterState(TERTIARY_UV);
            }
            break;

        case TERTIARY_UV:
            if (millis() - state_timer_start >= (unsigned long)currentProcess.tertiary_uv_exposure_sec * 1000) {
                digitalWrite(UV_380_PIN, LOW);
                digitalWrite(UV_430_PIN, LOW);
                Serial.println("Tertiary UV finished.");
                enterState(POST_COOLING); // Теперь отсюда переходим в охлаждение
            }
            break;
            
        case POST_COOLING:
            if (millis() - state_timer_start >= 60000) { // 60 секунд
                Serial.println("Post cooling finished. Process complete.");
                SerialPort.println("PROCESS_COMPLETE");
                enterState(IDLE); // Возвращаемся в ожидание
            }
            break;
            
        default:
            break;
    }
}


//==========================================================================
// Setup и Loop
//==========================================================================

void setup() {
    Serial.begin(115200);
    SerialPort.begin(115200, SERIAL_8N1, MAIN_BOARD_RX_PIN, MAIN_BOARD_TX_PIN);
    Serial.println("\n--- Sensor & Periphery Board Booting Up ---");

    // Настройка пинов MOSFET'ов
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(UV_380_PIN, OUTPUT);
    pinMode(UV_430_PIN, OUTPUT);

    // Гарантированное выключение всего при старте
    all_off();

    // Инициализация датчиков
    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
    tempSensor.begin();
    if (tempSensor.getDeviceCount() > 0) {
        ds18b20_found = true;
        tempSensor.setResolution(12);
        Serial.println("DS18B20 Temperature sensor: OK");
    } else {
        Serial.println("!!! DS18B20 Temperature sensor: NOT FOUND");
    }

    Wire.begin(O2_SDA_PIN, O2_SCL_PIN);
    if (gasSensor.begin()) {
        o2_sensor_found = true;
        gasSensor.changeAcquireMode(gasSensor.PASSIVITY);
        Serial.println("O2 Oxygen sensor: OK");
    } else {
        Serial.println("!!! O2 Oxygen sensor: NOT FOUND");
    }

    Serial.println("------------------------------------------");
    Serial.println("Setup complete. Entering IDLE state.");
    enterState(IDLE);
}

void loop() {
    // 1. Проверяем, пришла ли команда от главной платы
    if (SerialPort.available()) {
        String command = SerialPort.readStringUntil('\n');
        command.trim();
        if (command.length() > 0) {
            Serial.printf("Received UART command: %s\n", command.c_str());
            handleCommand(command);
        }
    }

    // 2. Выполняем текущую логику машины состояний
    state_machine_loop();
}