//==========================================================================
// Библиотеки
//==========================================================================
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_MultiGasSensor.h"

//==========================================================================
// Конфигурация пинов и устройств (НОВАЯ ВЕРСИЯ)
//==========================================================================

// --- UART для связи с главной платой (экраном) ---
#define MAIN_BOARD_RX_PIN 17
#define MAIN_BOARD_TX_PIN 16
HardwareSerial SerialPort(2);

// --- Датчики ---
#define DOOR_SENSOR_PIN     19  // Магнитный датчик двери
#define ONE_WIRE_BUS_PIN    23  // Датчик температуры DS18B20
#define O2_SDA_PIN          21  // Датчик кислорода O2 (I2C SDA)
#define O2_SCL_PIN          22  // Датчик кислорода O2 (I2C SCL)

// --- Пины MOSFET для управления периферией ---
// Старая система с LED-драйверами полностью заменена на прямое управление.
#define HEATER_PIN          13  // MOSFET: Нагревательный элемент ("Дуйка")
#define BLOWER_FAN_PIN      33  // MOSFET: Вентилятор циркуляции ("Вентилятор дуйки")
#define UV_380_PIN          14  // MOSFET: УФ-светодиоды 380 нм
#define UV_430_PIN          27  // MOSFET: УФ-светодиоды 430 нм
#define SYSTEM_COOLING_PIN  25  // MOSFET: Вентиляторы охлаждения системы (БП и т.д.)
#define MOTOR_PIN           32  // MOSFET: Мотор поворотного столика
#define AIR_VALVE_PIN       4   // MOSFET: Клапан сжатого воздуха (для охлаждения)
#define NITROGEN_VALVE_PIN  18  // MOSFET: Клапан азота (N2)
#define REPAIR_UV_PIN       5   // MOSFET: УФ-диод для режима "Ремонт" ("Склейка")

// --- Объекты датчиков ---
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature tempSensor(&oneWire);
DFRobot_GAS_I2C gasSensor(&Wire, 0x74);
bool ds18b20_found = false;
bool o2_sensor_found = false;
unsigned long last_temp_request_time = 0;
float last_known_temp = -127.0;

//==========================================================================
// Машина состояний и структура данных процесса (БЕЗ ИЗМЕНЕНИЙ)
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

// --- НОВЫЕ СТРУКТУРЫ ДЛЯ ЛАБОРАТОРНЫХ РЕЖИМОВ ---
struct LabParameters {
    // Общие
    bool use_cooling;
    // Глазурь
    float uv_on_sec;
    float uv_off_sec;
    int monomer_blow_min;
    int uv_exposure_sec;
    bool use_nitrogen;
    int nitrogen_target_percent;
    // Ремонт
    int countdown_sec;
    // Повышение прочности, Термокамера, Осветление
    int chamber_temp_c;
    int hold_time_min;
    int uv_pulse_duration_sec;
    int uv_pulse_interval_min;
    // Затемнение
    int darken_uv_exposure_min;
};
LabParameters currentLabProcess;

// --- НОВЫЕ СОСТОЯНИЯ ДЛЯ ЛАБОРАТОРНОЙ МАШИНЫ ---
enum LabProcessState {
    LAB_IDLE,
    LAB_PREPARATION,
    LAB_PRE_COOLING,
    LAB_MONOMER_BLOW,
    LAB_NITROGEN_PURGE,
    LAB_UV_FLICKER,
    LAB_COUNTDOWN,
    LAB_REPAIR_UV,
    LAB_HEATING,
    LAB_HOLDING_TEMP,
    LAB_HOLDING_TEMP_WITH_UV,
    LAB_STATIC_UV,
    LAB_FINAL_COOLING,
    LAB_FINISHING
};
LabProcessState currentLabState = LAB_IDLE;

// --- НОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ ЛАБОРАТОРНОЙ ЛОГИКИ ---
unsigned long lab_state_timer_start = 0;
unsigned long lab_uv_pulse_timer_start = 0;
bool lab_uv_flicker_led_state = false;
unsigned long lab_last_flicker_time = 0;
uint32_t lab_flicker_on_ms = 0;
uint32_t lab_flicker_off_ms = 0;
bool lab_door_check_enabled = true;

// --- Переменные для таймеров и проверок ---
unsigned long state_timer_start = 0;
unsigned long error_check_timer_start = 0;
float value_at_error_check_start = 0.0f;
unsigned long last_telemetry_send_time = 0;

// --- НОВЫЕ ПЕРЕМЕННЫЕ для логики мерцания UV ---
bool primary_uv_led_state = false;
unsigned long last_flicker_toggle_time = 0;
uint32_t flicker_half_period_ms = 0;

// --- Виртуальное состояние драйверов --- // <-- СЕКЦИЯ УДАЛЕНА

//==========================================================================
// Вспомогательные функции управления периферией (ПЕРЕРАБОТАНЫ)
//==========================================================================

/** @brief Выключает АБСОЛЮТНО ВСЮ силовую периферию. Функция безопасности. */
void all_off() {
    Serial.println("!!! EMERGENCY/ALL OFF triggered !!!");
    // Прямое управление всеми MOSFET'ами
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(BLOWER_FAN_PIN, LOW);
    digitalWrite(UV_380_PIN, LOW);
    digitalWrite(UV_430_PIN, LOW);
    digitalWrite(SYSTEM_COOLING_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    digitalWrite(AIR_VALVE_PIN, LOW);
    digitalWrite(NITROGEN_VALVE_PIN, LOW);
    digitalWrite(REPAIR_UV_PIN, LOW);
}

// --- Функции для опроса датчиков (БЕЗ ИЗМЕНЕНИЙ) ---
float readTemperature() {
    if (!ds18b20_found) return -127.0;
    
    if (tempSensor.isConversionComplete()) {
        last_known_temp = tempSensor.getTempCByIndex(0);
    }
    return last_known_temp;
}

float readOxygenLevel() {
    if (!o2_sensor_found) {
        return 20.8f; 
    }
    float o2_concentration = gasSensor.readGasConcentrationPPM();
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

void enterState(ProcessState newState);
void enterLabState(LabProcessState newState, int mode_id = 0);

// --- Функции телеметрии и обработки команд (БЕЗ ИЗМЕНЕНИЙ В ЛОГИКЕ) ---
void sendTelemetry() {
    StaticJsonDocument<256> doc;
    char buffer[256];
    doc["type"] = "TELEMETRY";
    doc["temp"] = readTemperature();
    doc["o2"] = readOxygenLevel();
    unsigned long elapsed_ms = millis() - state_timer_start;
    int remaining_sec = 0;
    switch(currentState) {
        case HEATING_HOLDING:
            if (heatingState == HS_HOLDING) {
                remaining_sec = currentProcess.heat_exchange_hold_sec - (elapsed_ms / 1000);
                doc["timer_rem"] = max(0, remaining_sec);
            } else {
                doc["timer_rem"] = -1;
            }
            break;
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
            doc["timer_rem"] = -1;
            break;
    }
    serializeJson(doc, buffer);
    SerialPort.println(buffer);
}

void sendLabTelemetry() {
    StaticJsonDocument<256> doc;
    char buffer[256];
    doc["type"] = "TELEMETRY";
    doc["temp"] = readTemperature();
    doc["o2"] = readOxygenLevel();
    unsigned long elapsed_ms = millis() - lab_state_timer_start;
    int remaining_sec = 0;
    switch(currentLabState) {
        case LAB_MONOMER_BLOW:
            remaining_sec = (currentLabProcess.monomer_blow_min * 60) - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case LAB_UV_FLICKER:
            remaining_sec = currentLabProcess.uv_exposure_sec - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case LAB_COUNTDOWN:
            remaining_sec = currentLabProcess.countdown_sec - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case LAB_REPAIR_UV:
            remaining_sec = currentLabProcess.uv_exposure_sec - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case LAB_HOLDING_TEMP:
        case LAB_HOLDING_TEMP_WITH_UV:
            remaining_sec = (currentLabProcess.hold_time_min * 60) - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case LAB_STATIC_UV:
             remaining_sec = (currentLabProcess.darken_uv_exposure_min * 60) - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        case LAB_FINISHING:
            remaining_sec = 60 - (elapsed_ms / 1000);
            doc["timer_rem"] = max(0, remaining_sec);
            break;
        default:
            doc["timer_rem"] = -1;
            break;
    }
    serializeJson(doc, buffer);
    SerialPort.println(buffer);
}

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
        if (currentState != IDLE) { return; }
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
        enterLabState(LAB_IDLE);
    } else if (strcmp(command, "START_LAB_PROCESS") == 0) {
        if (currentState != IDLE || currentLabState != LAB_IDLE) { return; }
        int mode_id = doc["mode_id"];
        JsonObject params = doc["params"];
        memset(&currentLabProcess, 0, sizeof(LabParameters));
        switch(mode_id) {
            case 1: // Глазурь
                currentLabProcess.uv_on_sec = params["uv_on_sec"];
                currentLabProcess.uv_off_sec = params["uv_off_sec"];
                currentLabProcess.monomer_blow_min = params["monomer_blow_min"];
                currentLabProcess.uv_exposure_sec = params["uv_exposure_sec"];
                currentLabProcess.use_cooling = params["use_cooling"];
                currentLabProcess.use_nitrogen = params["use_nitrogen"];
                currentLabProcess.nitrogen_target_percent = params["nitrogen_target_percent"];
                break;
            case 2: // Ремонт
                currentLabProcess.countdown_sec = params["countdown_sec"];
                currentLabProcess.uv_exposure_sec = params["uv_exposure_sec"];
                break;
            case 3: // Повышение прочности
                currentLabProcess.chamber_temp_c = params["chamber_temp_c"];
                currentLabProcess.hold_time_min = params["hold_time_min"];
                currentLabProcess.use_cooling = params["use_cooling"];
                currentLabProcess.uv_pulse_duration_sec = params["uv_pulse_duration_sec"];
                currentLabProcess.uv_pulse_interval_min = params["uv_pulse_interval_min"];
                break;
            case 4: // Термокамера
                currentLabProcess.chamber_temp_c = params["chamber_temp_c"];
                currentLabProcess.hold_time_min = params["hold_time_min"];
                break;
            case 5: // Осветление
                currentLabProcess.chamber_temp_c = params["chamber_temp_c"];
                currentLabProcess.hold_time_min = params["hold_time_min"];
                currentLabProcess.use_cooling = params["use_cooling"];
                break;
            case 6: // Затемнение
                currentLabProcess.darken_uv_exposure_min = params["darken_uv_exposure_min"];
                currentLabProcess.use_cooling = params["use_cooling"];
                break;
        }
        Serial.printf("Received START_LAB_PROCESS command for mode %d. Entering LAB_PREPARATION state.\n", mode_id);
        SerialPort.println("ACK:PROCESS_STARTED");
        enterLabState(LAB_PREPARATION, mode_id);
    }
}

/** @brief Функция смены состояния. Здесь описывается, что делать при ВХОДЕ в новое состояние. (ПЕРЕРАБОТАНО) */
void enterState(ProcessState newState) {
    currentState = newState;
    state_timer_start = millis();

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
            digitalWrite(SYSTEM_COOLING_PIN, HIGH); 
            digitalWrite(MOTOR_PIN, HIGH);
            SerialPort.println("STATUS:PREPARATION_OK");
            enterState(HEATING_HOLDING);
            break;

        case HEATING_HOLDING: {
            Serial.println("Entering HEATING_HOLDING state.");
            if (!currentProcess.thermal_chamber_enabled) {
                Serial.println("Thermal chamber disabled. Skipping.");
                heatingState = HS_IDLE;
                enterState(NITROGEN_PURGE);
                return;
            }
            float currentTemp = readTemperature();
            float targetTemp = currentProcess.thermal_chamber_temp;
            if (currentTemp < targetTemp - 0.5) {
                heatingState = HS_HEATING;
                digitalWrite(BLOWER_FAN_PIN, HIGH);
                digitalWrite(HEATER_PIN, HIGH);
                error_check_timer_start = millis();
                value_at_error_check_start = currentTemp;
                SerialPort.println("STATUS:HEATING_STARTED");
            } else if (currentTemp > targetTemp + 0.5 && currentProcess.chamber_cooling_enabled) {
                heatingState = HS_COOLING;
                digitalWrite(AIR_VALVE_PIN, HIGH);
                error_check_timer_start = millis();
                value_at_error_check_start = currentTemp;
                SerialPort.println("STATUS:COOLING_STARTED");
            } else {
                heatingState = HS_HOLDING;
                state_timer_start = millis();
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
            digitalWrite(NITROGEN_VALVE_PIN, HIGH);
            error_check_timer_start = millis();
            value_at_error_check_start = readOxygenLevel();
            SerialPort.println("STATUS:NITROGEN_PURGE_STARTED");
            break;

        case PRIMARY_UV:
            Serial.println("Entering PRIMARY_UV state.");
            SerialPort.println("STATUS:PRIMARY_UV_STARTED");
            digitalWrite(UV_380_PIN, LOW);
            digitalWrite(UV_430_PIN, LOW);
            primary_uv_led_state = false;
            flicker_half_period_ms = 0;
            if (currentProcess.primary_uv_flicker_rate > 0) {
                flicker_half_period_ms = (1000 / currentProcess.primary_uv_flicker_rate) / 2;
                if (flicker_half_period_ms == 0) flicker_half_period_ms = 1;
            }
            last_flicker_toggle_time = state_timer_start;
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
            SerialPort.println("STATUS:TERTIARY_UV_STARTED");
            break;
            
        case POST_COOLING:
            Serial.println("Entering POST_COOLING state.");
            digitalWrite(MOTOR_PIN, LOW);
            digitalWrite(NITROGEN_VALVE_PIN, LOW);
            digitalWrite(BLOWER_FAN_PIN, HIGH); // Используем "дуйку" для охлаждения
            SerialPort.println("STATUS:POST_COOLING_STARTED");
            break;
    }
}

/** @brief Основная логика, выполняющаяся в каждом цикле (ПЕРЕРАБОТАНО) */
void state_machine_loop() {
    if (currentState != IDLE && millis() - last_telemetry_send_time > 1000) {
        sendTelemetry();
        last_telemetry_send_time = millis();
    }
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
        case IDLE: break;
        case HEATING_HOLDING: {
            if (heatingState == HS_IDLE) break;
            float currentTemp = readTemperature();
            float targetTemp = currentProcess.thermal_chamber_temp;
            switch (heatingState) {
                case HS_HEATING:
                    if (currentTemp >= targetTemp) {
                        Serial.println("Target temperature reached. Starting hold timer.");
                        digitalWrite(HEATER_PIN, LOW);
                        digitalWrite(BLOWER_FAN_PIN, LOW); // Выключаем вентилятор, если нагрев не нужен
                        state_timer_start = millis();
                        heatingState = HS_HOLDING;
                        SerialPort.println("STATUS:HOLDING_TEMPERATURE");
                        break;
                    }
                    if (millis() - error_check_timer_start > 100000) {
                        if (currentTemp - value_at_error_check_start < 1.0f) {
                            Serial.println("!!! FATAL ERROR: HEATER FAILURE !!!");
                            all_off();
                            SerialPort.println("FATAL_ERROR:HEATER_FAILURE");
                            enterState(IDLE);
                        } else {
                            error_check_timer_start = millis();
                            value_at_error_check_start = currentTemp;
                        }
                    }
                    break;
                case HS_COOLING:
                    if (currentTemp <= targetTemp) {
                        Serial.println("Target temperature reached. Starting hold timer.");
                        digitalWrite(AIR_VALVE_PIN, LOW);
                        state_timer_start = millis();
                        heatingState = HS_HOLDING;
                        SerialPort.println("STATUS:HOLDING_TEMPERATURE");
                        break;
                    }
                    if (millis() - error_check_timer_start > 100000) {
                        if (value_at_error_check_start - currentTemp < 1.0f) {
                            Serial.println("Warning: Cooling timed out. Skipping.");
                            digitalWrite(AIR_VALVE_PIN, LOW);
                            heatingState = HS_IDLE;
                            SerialPort.println("WARN:COOLING_SKIPPED");
                            enterState(NITROGEN_PURGE);
                        } else {
                            error_check_timer_start = millis();
                            value_at_error_check_start = currentTemp;
                        }
                    }
                    break;
                case HS_HOLDING:
                    if (currentTemp < targetTemp - 0.5) {
                        digitalWrite(HEATER_PIN, HIGH);
                        digitalWrite(BLOWER_FAN_PIN, HIGH);
                    } else {
                        digitalWrite(HEATER_PIN, LOW);
                        digitalWrite(BLOWER_FAN_PIN, LOW);
                    }
                    if (millis() - state_timer_start >= (unsigned long)currentProcess.heat_exchange_hold_sec * 1000) {
                        Serial.println("Hold time finished.");
                        all_off();
                        digitalWrite(SYSTEM_COOLING_PIN, HIGH);
                        digitalWrite(MOTOR_PIN, HIGH);
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
                digitalWrite(NITROGEN_VALVE_PIN, LOW);
                enterState(PRIMARY_UV);
                return;
            }
            if (millis() - error_check_timer_start > 100000) {
                if (value_at_error_check_start - currentO2 < 1.0f) {
                    Serial.println("Warning: Nitrogen purge timed out. Skipping.");
                    digitalWrite(NITROGEN_VALVE_PIN, LOW);
                    SerialPort.println("WARN:NITROGEN_SKIPPED");
                    enterState(PRIMARY_UV);
                } else {
                    error_check_timer_start = millis();
                    value_at_error_check_start = currentO2;
                }
            }
            break;
        }
        case PRIMARY_UV: {
            if (millis() - state_timer_start >= (unsigned long)currentProcess.primary_uv_exposure_sec * 1000) {
                Serial.println("Primary UV finished.");
                enterState(SECONDARY_UV);
                return; 
            }
            if (flicker_half_period_ms > 0) {
                if (millis() - last_flicker_toggle_time >= flicker_half_period_ms) {
                    primary_uv_led_state = !primary_uv_led_state; 
                    if (primary_uv_led_state) { 
                        if (currentProcess.primary_uv_mode == 0 || currentProcess.primary_uv_mode == 2) digitalWrite(UV_380_PIN, HIGH);
                        if (currentProcess.primary_uv_mode == 1 || currentProcess.primary_uv_mode == 2) digitalWrite(UV_430_PIN, HIGH);
                    } else { 
                        digitalWrite(UV_380_PIN, LOW);
                        digitalWrite(UV_430_PIN, LOW);
                    }
                    last_flicker_toggle_time = millis(); 
                }
            } else {
                if (!primary_uv_led_state) {
                    if (currentProcess.primary_uv_mode == 0 || currentProcess.primary_uv_mode == 2) digitalWrite(UV_380_PIN, HIGH);
                    if (currentProcess.primary_uv_mode == 1 || currentProcess.primary_uv_mode == 2) digitalWrite(UV_430_PIN, HIGH);
                    primary_uv_led_state = true;
                }
            }
            break;
        }
        case SECONDARY_UV:
            if (millis() - state_timer_start >= (unsigned long)currentProcess.secondary_uv_exposure_sec * 1000) {
                Serial.println("Secondary UV finished.");
                enterState(TERTIARY_UV);
            }
            break;
        case TERTIARY_UV:
            if (millis() - state_timer_start >= (unsigned long)currentProcess.tertiary_uv_exposure_sec * 1000) {
                digitalWrite(UV_380_PIN, LOW);
                digitalWrite(UV_430_PIN, LOW);
                Serial.println("Tertiary UV finished.");
                enterState(POST_COOLING);
            }
            break;
        case POST_COOLING:
            if (millis() - state_timer_start >= 60000) {
                Serial.println("Post cooling finished. Process complete.");
                SerialPort.println("PROCESS_COMPLETE");
                enterState(IDLE);
            }
            break;
        default: break;
    }
}

/** @brief Логика лабораторной машины состояний (ПЕРЕРАБОТАНО) */
void enterLabState(LabProcessState newState, int mode_id) {
    static int last_mode_id = 0;
    if (mode_id != 0) { last_mode_id = mode_id; } else { mode_id = last_mode_id; }
    currentLabState = newState;
    lab_state_timer_start = millis();
    all_off();
    digitalWrite(SYSTEM_COOLING_PIN, HIGH);
    if (mode_id != 2) digitalWrite(MOTOR_PIN, HIGH);

    switch (currentLabState) {
        case LAB_IDLE:
            Serial.println("Entering LAB_IDLE state.");
            all_off();
            break;
        case LAB_PREPARATION:
            Serial.println("Entering LAB_PREPARATION state.");
            lab_door_check_enabled = (mode_id != 2);
            if (lab_door_check_enabled && !isDoorClosed()) {
                Serial.println("Error: Door is open. Aborting lab process.");
                SerialPort.println("ERROR:DOOR_IS_OPEN");
                enterLabState(LAB_IDLE);
                return;
            }
            SerialPort.println("STATUS:PREPARATION_OK");
            switch(mode_id) {
                case 1: case 6: enterLabState(LAB_PRE_COOLING); break;
                case 2: enterLabState(LAB_COUNTDOWN); break;
                case 3: case 4: case 5: enterLabState(LAB_HEATING); break;
                default: enterLabState(LAB_IDLE); break;
            }
            break;
        case LAB_PRE_COOLING:
             Serial.println("Entering LAB_PRE_COOLING state.");
             if (currentLabProcess.use_cooling && readTemperature() > 40.0f) {
                 digitalWrite(AIR_VALVE_PIN, HIGH);
                 SerialPort.println("STATUS:Pre-cooling chamber...");
             } else {
                 if(mode_id == 1) enterLabState(LAB_MONOMER_BLOW);
                 if(mode_id == 6) enterLabState(LAB_STATIC_UV);
             }
             break;
        case LAB_MONOMER_BLOW:
            Serial.println("Entering LAB_MONOMER_BLOW state.");
            digitalWrite(BLOWER_FAN_PIN, HIGH);
            SerialPort.println("STATUS:Blowing monomer...");
            break;
        case LAB_NITROGEN_PURGE:
            Serial.println("Entering LAB_NITROGEN_PURGE state.");
            if (currentLabProcess.use_nitrogen) {
                digitalWrite(NITROGEN_VALVE_PIN, HIGH);
                SerialPort.println("STATUS:Purging with nitrogen...");
            } else {
                enterLabState(LAB_UV_FLICKER);
            }
            break;
        case LAB_UV_FLICKER:
            Serial.println("Entering LAB_UV_FLICKER state.");
            lab_flicker_on_ms = (uint32_t)(currentLabProcess.uv_on_sec * 1000.0f);
            lab_flicker_off_ms = (uint32_t)(currentLabProcess.uv_off_sec * 1000.0f);
            lab_uv_flicker_led_state = false;
            lab_last_flicker_time = millis();
            SerialPort.println("STATUS:UV flickering started...");
            break;
        case LAB_COUNTDOWN:
            Serial.println("Entering LAB_COUNTDOWN state.");
            SerialPort.printf("EVENT:START_COUNTDOWN:%d\n", currentLabProcess.countdown_sec);
            break;
        case LAB_REPAIR_UV:
            Serial.println("Entering LAB_REPAIR_UV state.");
            digitalWrite(REPAIR_UV_PIN, HIGH);
            SerialPort.println("STATUS:Repair UV is ON");
            break;
        case LAB_HEATING:
            Serial.println("Entering LAB_HEATING state.");
            digitalWrite(HEATER_PIN, HIGH);
            digitalWrite(BLOWER_FAN_PIN, HIGH);
            SerialPort.println("STATUS:Heating chamber...");
            break;
        case LAB_HOLDING_TEMP:
            Serial.println("Entering LAB_HOLDING_TEMP state.");
            SerialPort.println("STATUS:Holding temperature...");
            break;
        case LAB_HOLDING_TEMP_WITH_UV:
            Serial.println("Entering LAB_HOLDING_TEMP_WITH_UV state.");
            lab_uv_pulse_timer_start = millis();
            SerialPort.println("STATUS:Holding temperature with UV pulses...");
            break;
        case LAB_STATIC_UV:
            Serial.println("Entering LAB_STATIC_UV state.");
            digitalWrite(UV_380_PIN, HIGH);
            SerialPort.println("STATUS:Static UV exposure...");
            break;
        case LAB_FINAL_COOLING:
            Serial.println("Entering LAB_FINAL_COOLING state.");
            if(currentLabProcess.use_cooling) {
                digitalWrite(AIR_VALVE_PIN, HIGH);
                SerialPort.println("STATUS:Final cooling...");
            } else {
                enterLabState(LAB_FINISHING);
            }
            break;
        case LAB_FINISHING:
            Serial.println("Entering LAB_FINISHING state (System Cooling).");
            all_off();
            digitalWrite(SYSTEM_COOLING_PIN, HIGH);
            lab_door_check_enabled = false;
            SerialPort.println("STATUS:System components cooling");
            break;
    }
}

/** @brief Основная логика лабораторной машины состояний. (ПЕРЕРАБОТАНО) */
void lab_state_machine_loop() {
    if (currentLabState != LAB_IDLE && millis() - last_telemetry_send_time > 500) {
        sendLabTelemetry();
        last_telemetry_send_time = millis();
    }
    if (currentLabState == LAB_IDLE) return;
    if (lab_door_check_enabled && !isDoorClosed()) {
        Serial.println("!!! DOOR OPENED DURING LAB PROCESS !!! EMERGENCY STOP !!!");
        all_off();
        SerialPort.println("EVENT:DOOR_OPENED_EMERGENCY_STOP");
        enterLabState(LAB_IDLE);
        return;
    }
    switch (currentLabState) {
        case LAB_PRE_COOLING:
            if (readTemperature() <= 40.0f) {
                if (currentLabProcess.darken_uv_exposure_min > 0) enterLabState(LAB_STATIC_UV);
                else enterLabState(LAB_MONOMER_BLOW);
            }
            break;
        case LAB_MONOMER_BLOW:
            if (millis() - lab_state_timer_start >= (unsigned long)currentLabProcess.monomer_blow_min * 60000UL) {
                enterLabState(LAB_NITROGEN_PURGE);
            }
            break;
        case LAB_NITROGEN_PURGE: {
            float targetO2 = 100.0f - currentLabProcess.nitrogen_target_percent;
            if (readOxygenLevel() <= targetO2) { enterLabState(LAB_UV_FLICKER); }
            if (millis() - lab_state_timer_start > 120000UL) {
                Serial.println("WARN: Nitrogen purge timeout");
                SerialPort.println("WARN:NITROGEN_SKIPPED");
                enterLabState(LAB_UV_FLICKER);
            }
            break;
        }
        case LAB_UV_FLICKER:
            if (millis() - lab_state_timer_start >= (unsigned long)currentLabProcess.uv_exposure_sec * 1000UL) {
                enterLabState(LAB_FINISHING);
                return;
            }
            if (!lab_uv_flicker_led_state) {
                if (millis() - lab_last_flicker_time >= lab_flicker_off_ms) {
                    digitalWrite(UV_380_PIN, HIGH);
                    lab_uv_flicker_led_state = true;
                    lab_last_flicker_time = millis();
                }
            } else {
                if (millis() - lab_last_flicker_time >= lab_flicker_on_ms) {
                    digitalWrite(UV_380_PIN, LOW);
                    lab_uv_flicker_led_state = false;
                    lab_last_flicker_time = millis();
                }
            }
            if (readTemperature() > 45.0f) digitalWrite(AIR_VALVE_PIN, HIGH); else digitalWrite(AIR_VALVE_PIN, LOW);
            break;
        case LAB_COUNTDOWN:
            if (millis() - lab_state_timer_start >= (unsigned long)currentLabProcess.countdown_sec * 1000UL) {
                enterLabState(LAB_REPAIR_UV);
            }
            break;
        case LAB_REPAIR_UV:
            if (millis() - lab_state_timer_start >= (unsigned long)currentLabProcess.uv_exposure_sec * 1000UL) {
                Serial.println("Repair UV finished. Process complete.");
                SerialPort.println("PROCESS_COMPLETE");
                enterLabState(LAB_IDLE);
            }
            break;
        case LAB_HEATING:
            if (readTemperature() >= currentLabProcess.chamber_temp_c) {
                if (currentLabProcess.uv_pulse_duration_sec > 0) enterLabState(LAB_HOLDING_TEMP_WITH_UV);
                else enterLabState(LAB_HOLDING_TEMP);
            }
            break;
        case LAB_HOLDING_TEMP:
            if (readTemperature() < currentLabProcess.chamber_temp_c - 0.5f) digitalWrite(HEATER_PIN, HIGH);
            else digitalWrite(HEATER_PIN, LOW);
            if (millis() - lab_state_timer_start >= (unsigned long)currentLabProcess.hold_time_min * 60000UL) {
                enterLabState(LAB_FINAL_COOLING);
            }
            break;
        case LAB_HOLDING_TEMP_WITH_UV:
            if (readTemperature() < currentLabProcess.chamber_temp_c - 0.5f) digitalWrite(HEATER_PIN, HIGH);
            else digitalWrite(HEATER_PIN, LOW);
            {
                unsigned long pulse_interval_ms = (unsigned long)currentLabProcess.uv_pulse_interval_min * 60000UL;
                unsigned long pulse_duration_ms = (unsigned long)currentLabProcess.uv_pulse_duration_sec * 1000UL;
                unsigned long elapsed_since_pulse_start = millis() - lab_uv_pulse_timer_start;
                if (elapsed_since_pulse_start >= pulse_interval_ms) {
                    digitalWrite(UV_430_PIN, HIGH);
                    if (elapsed_since_pulse_start >= pulse_interval_ms + pulse_duration_ms) {
                        digitalWrite(UV_430_PIN, LOW);
                        lab_uv_pulse_timer_start = millis();
                    }
                }
            }
            if (millis() - lab_state_timer_start >= (unsigned long)currentLabProcess.hold_time_min * 60000UL) {
                enterLabState(LAB_FINAL_COOLING);
            }
            break;
        case LAB_STATIC_UV:
            if(currentLabProcess.use_cooling) {
                if (readTemperature() > 40.0f) digitalWrite(AIR_VALVE_PIN, HIGH); else digitalWrite(AIR_VALVE_PIN, LOW);
            }
            if (millis() - lab_state_timer_start >= (unsigned long)currentLabProcess.darken_uv_exposure_min * 60000UL) {
                enterLabState(LAB_FINISHING);
            }
            break;
        case LAB_FINAL_COOLING:
            if (readTemperature() <= 45.0f) { enterLabState(LAB_FINISHING); }
            if (millis() - lab_state_timer_start > 180000UL) { enterLabState(LAB_FINISHING); }
            break;
        case LAB_FINISHING:
            if (millis() - lab_state_timer_start >= 60000UL) {
                Serial.println("System cooling finished. Process complete.");
                SerialPort.println("PROCESS_COMPLETE");
                enterLabState(LAB_IDLE);
            }
            break;
        default: break;
    }
}

//==========================================================================
// Setup и Loop (ПЕРЕРАБОТАНО)
//==========================================================================

void setup() {
    Serial.begin(115200);
    SerialPort.begin(115200, SERIAL_8N1, MAIN_BOARD_RX_PIN, MAIN_BOARD_TX_PIN);
    Serial.println("\n--- Sensor & Periphery Board Booting Up (v2.0 - Direct MOSFET Control) ---");

    // Настройка пинов MOSFET'ов как ВЫХОДОВ
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(BLOWER_FAN_PIN, OUTPUT);
    pinMode(UV_380_PIN, OUTPUT);
    pinMode(UV_430_PIN, OUTPUT);
    pinMode(SYSTEM_COOLING_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(AIR_VALVE_PIN, OUTPUT);
    pinMode(NITROGEN_VALVE_PIN, OUTPUT);
    pinMode(REPAIR_UV_PIN, OUTPUT);
    
    // Гарантированное выключение всего при старте
    all_off();

    // Инициализация датчиков
    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
    tempSensor.begin();
    if (tempSensor.getDeviceCount() > 0) {
        ds18b20_found = true;
        tempSensor.setResolution(12);
        tempSensor.setWaitForConversion(false);
        Serial.println("DS18B20 Temperature sensor: OK (non-blocking mode)");
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

    // 2. НЕБЛОКИРУЮЩИЙ ЗАПРОС ТЕМПЕРАТУРЫ
    if (ds18b20_found && (millis() - last_temp_request_time > 1000)) {
        tempSensor.requestTemperatures();
        last_temp_request_time = millis();
    }

    // 3. Выполняем текущую логику машины состояний
    state_machine_loop();
    lab_state_machine_loop();
}