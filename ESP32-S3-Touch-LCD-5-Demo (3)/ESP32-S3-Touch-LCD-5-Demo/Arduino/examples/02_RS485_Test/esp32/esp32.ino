// Код для второй платы (обычная ESP32)

#include <HardwareSerial.h>

// --- ИЗМЕНЕНИЕ ЗДЕСЬ: Новые, "безопасные" пины ---
#define RX2_PIN 21 
#define TX2_PIN 22

// Здесь можно оставить UART1, он не конфликтует на этой плате
HardwareSerial MySerial2(1);

void setup() {
  Serial.begin(115200);
  Serial.println("Обычная ESP32 #2 - Прием на пинах 21/22");

  // Инициализируем наш UART1 на НОВЫХ пинах
  MySerial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
}

void loop() {
  // --- Прием данных ---
  if (MySerial2.available()) {
    String receivedMessage = MySerial2.readStringUntil('\n');
    Serial.print("Получено от S3: ");
    Serial.println(receivedMessage);
    
    // --- Отправка ответа ---
    String reply = "Обычная ESP32 подтверждает получение!";
    MySerial2.println(reply);
    Serial.print("Отправлен ответ: ");
    Serial.println(reply);
  }
}