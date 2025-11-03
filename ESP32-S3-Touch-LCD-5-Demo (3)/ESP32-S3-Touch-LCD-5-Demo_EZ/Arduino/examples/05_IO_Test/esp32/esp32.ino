#include <Arduino.h>

// Пины для вашего TTL-конвертера
#define RS485_RX_PIN 16 
#define RS485_TX_PIN 17
#define RS485 Serial2
// Если ваш конвертер требует ручного управления DE/RE
// #define RS485_DE_RE_PIN 4 

void setup() {
  Serial.begin(115200);
  Serial.println("--- RS485 Передатчик ('голый' ESP32) запущен ---");
  RS485.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  // Если есть ручное управление:
  // pinMode(RS485_DE_RE_PIN, OUTPUT);
  // digitalWrite(RS485_DE_RE_PIN, LOW); // По умолчанию в приеме
}

void loop() {
  Serial.println("Отправка сообщения от 'голого' ESP32...");
  // Если есть ручное управление:
  // digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(200);

  RS485.println("Hello from Naked ESP32!");
  RS485.flush();
  
  delayMicroseconds(200);
  // digitalWrite(RS485_DE_RE_PIN, LOW);

  delay(2000);
}