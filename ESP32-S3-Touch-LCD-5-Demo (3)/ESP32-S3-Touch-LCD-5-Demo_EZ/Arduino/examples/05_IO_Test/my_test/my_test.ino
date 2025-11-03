// Определяем пины для связи
#define WAVESHARE_TX_PIN 6  // DO0
#define WAVESHARE_RX_PIN 4  // DI0

void setup() {
  // Serial для вывода в монитор порта на компьютере
  Serial.begin(115200);
  Serial.println("Waveshare Board: UART Sender");

  // Инициализируем второй UART-порт (Serial1) на наших пинах
  // Формат: Serial1.begin(скорость, режим, RX_PIN, TX_PIN);
  Serial1.begin(9600, SERIAL_8N1, WAVESHARE_RX_PIN, WAVESHARE_TX_PIN);
}

void loop() {
  // Отправляем сообщение на вторую ESP32 каждые 2 секунды
  String message = "Привет от Waveshare! Время: " + String(millis());
  Serial1.println(message);
  
  // Выводим в локальный монитор то, что отправили
  Serial.print("Отправлено: ");
  Serial.println(message);

  delay(2000);
}