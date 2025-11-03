// --- Код для ОБЫЧНОЙ ESP32, работающий через UART0 (GPIO 1, 3) ---
// ВАЖНО: Монитор порта для этой платы работать не будет!

// Мы не используем Serial2. Мы будем использовать основной порт Serial.
// #define RS485_SERIAL Serial // Это и так по умолчанию

long messageCounter = 0;

void setup() {
  // Инициализируем основной порт UART0 (GPIO 1, 3)
  // Вся отладка через Serial.println() теперь будет уходить в RS485!
  Serial.begin(115200); 
  
  // Никаких Serial.println() здесь быть не должно!
  delay(100);
}

void loop() {
  // Формируем сообщение
  String messageToSend = "Success via UART0, count: " + String(messageCounter);
  
  // Отправляем сообщение по RS485 через UART0
  Serial.println(messageToSend);

  messageCounter++;

  // Ждем 2 секунды перед следующей отправкой
  delay(2000); 
}