// =================== НАСТРОЙКА ===================
// Раскомментируйте эту строку для ПЛАТЫ 1 (Мастер)
// Для ПЛАТЫ 2 (Слейв) оставьте эту строку закомментированной
#define MASTER_NODE

// =================== КОД ПРОГРАММЫ ===================

// Определяем пины для RS485
#define RS485_RX_PIN  43
#define RS485_TX_PIN  44

// Псевдоним для Serial1
#define RS485 Serial1

// Буфер для приема данных и флаги
char receivedChars[128];
boolean newData = false;

void setup() {
  // Инициализируем Serial для отладки в Мониторе порта
  Serial.begin(115200);
  while(!Serial); // Ожидание открытия порта
  
  // Инициализируем RS485
  RS485.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  delay(100);

  #ifdef MASTER_NODE
    Serial.println("--- Плата Мастер запущена ---");
    Serial.println("Отправляю первый PING через 3 секунды...");
  #else
    Serial.println("--- Плата Слейв запущена ---");
    Serial.println("Ожидаю PING от Мастера...");
  #endif

  delay(3000);
}

void loop() {
  // Функции для приема и обработки данных
  receiveFromRS485();
  processData();

  #ifdef MASTER_NODE
    // Мастер отправляет "PING" каждые 5 секунд
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime > 5000) {
      lastSendTime = millis();
      RS485.println("PING");
      RS485.flush(); // Важно! Гарантирует отправку данных перед переключением режима
      Serial.println("-> Отправлено: PING");
    }
  #endif
}

// Функция для чтения данных из RS485 в буфер
void receiveFromRS485() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (RS485.available() > 0 && newData == false) {
    rc = RS485.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= sizeof(receivedChars)) {
        ndx = sizeof(receivedChars) - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // Завершаем строку
      ndx = 0;
      newData = true;
    }
  }
}

// Функция для обработки полученных данных
void processData() {
  if (newData == true) {
    // Убираем возможный символ \r из конца строки
    if (receivedChars[strlen(receivedChars) - 1] == '\r') {
        receivedChars[strlen(receivedChars) - 1] = '\0';
    }

    Serial.print("<- Получено: ");
    Serial.println(receivedChars);

    #ifndef MASTER_NODE
      // Если мы Слейв и получили PING, отвечаем PONG
      if (strcmp(receivedChars, "PING") == 0) {
        delay(10); // Небольшая пауза, чтобы Мастер успел переключиться на прием
        RS485.println("PONG");
        RS485.flush(); // Гарантируем отправку
        Serial.println("-> Отвечено: PONG");
      }
    #endif

    newData = false; // Сбрасываем флаг
  }
}