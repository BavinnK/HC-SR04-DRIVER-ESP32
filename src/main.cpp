volatile unsigned long start_time = 0;
#define trigPin 5
#define echopin 18
#define F_PIN 32
#define CH_0 0
#define res 8
#define freq 20000



SemaphoreHandle_t emergency_stop;
QueueHandle_t distanceQueue;

void ultra_sonic_task(void *pvParameters) {


  while (1) {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void IRAM_ATTR echo_ISR() {
  bool echo_stat = digitalRead(echopin);

  if (echo_stat == HIGH) {
    start_time = micros();
  } else {
    unsigned long pulse_width = (micros() - start_time);
    BaseType_t taskWoken = pdFALSE;

    xQueueSendFromISR(distanceQueue, &pulse_width, &taskWoken);
    if (taskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}
void motor_task(void *pvParameters) {

  ledcSetup(CH_0, freq, res);
  ledcAttachPin(F_PIN, CH_0);
  unsigned long distance_raw;
  for (;;) {
    if (xSemaphoreTake(emergency_stop, 0) == pdTRUE) {
      Serial.println("Emergency stop activated, motors are off! ");
      ledcWrite(CH_0, 0);
      while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    if (xQueueReceive(distanceQueue, &distance_raw, 10) == pdTRUE) {
      unsigned long distance_data_cm = distance_raw / 58;
      if (distance_data_cm > 20) {
        ledcWrite(CH_0, 150);
      } else {
        ledcWrite(CH_0, 0);
      }
    }
  }
}
void emergency_stop_task(void *pvParameters) {
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  Serial.println("Emergency stop activated, Termination begun! ");
  xSemaphoreGive(emergency_stop);
  vTaskDelete(NULL);
}
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echopin, INPUT);
  attachInterrupt(echopin, echo_ISR, CHANGE);
  emergency_stop = xSemaphoreCreateBinary();
  distanceQueue = xQueueCreate(10, sizeof(unsigned long));
  Serial.begin(115200);
  xTaskCreatePinnedToCore(emergency_stop_task, "EmergencyStopTask", 1096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(ultra_sonic_task, "UltraSonicTask", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(motor_task, "MotorRunningTask", 4096, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}
