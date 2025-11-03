// Код для первой платы (ESP32-S3)
// с кастомной комбинацией пинов

#include <HardwareSerial.h>

// --- ИЗМЕНЕНИЕ ЗДЕСЬ ---
// Назначаем RX на новый пин, а TX оставляем на 44
#define RX1_PIN 15 
#define TX1_PIN 44
// ----------------------

// По-прежнему используем UART2 для максимальной надежности
HardwareSerial MySerial1(2);

unsigned long previousMillis = 0;
const long interval = 2000;
int counter = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 #1 - Отправка с TX=44, прием на RX=15");

  // Инициализируем наш UART2 на этой новой паре пинов
  MySerial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
}

void loop() {
  // --- Отправка данных ---
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    String messageToSend = "Привет от S3! Счетчик: " + String(counter);
    MySerial1.println(messageToSend);
    Serial.print("Отправлено: ");
    Serial.println(messageToSend);
    counter++;
  }

  // --- Прием данных ---
  if (MySerial1.available()) {
    String receivedMessage = MySerial1.readStringUntil('\n');
    Serial.print("Получен ответ: ");
    Serial.println(receivedMessage);
  }
}