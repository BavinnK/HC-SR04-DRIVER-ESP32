const int trigPin = 5, echopin = 18;
volatile unsigned long start_time = 0;
volatile unsigned long pulse_width = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void ultra_sonic_task(void *pvParameters) {


  while (1) {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}
void IRAM_ATTR echo_ISR() {
  bool echo_stat = digitalRead(echopin);
  portENTER_CRITICAL_ISR(&timerMux);
  if (echo_stat==HIGH) {
    start_time = micros();
  }
  else {
   pulse_width = micros()-start_time;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echopin, INPUT);
  attachInterrupt(echopin, echo_ISR, CHANGE);
  Serial0.begin(115200);
  xTaskCreatePinnedToCore(ultra_sonic_task, "ultraSonicTask", 4096, NULL, 2, NULL, 0);

}

void loop() {
  char buff [70];
  portENTER_CRITICAL_ISR(&timerMux);
  uint16_t distance_raw=pulse_width;
  portEXIT_CRITICAL_ISR(&timerMux);
  uint16_t distance_data=distance_raw/58;

  sprintf(buff, "Distance in cm: %d",distance_data);
  Serial0.println(buff);
  delay(100);



}
