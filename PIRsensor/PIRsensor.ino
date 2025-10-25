
const int PIR_SENSOR_OUTPUT_PIN = 18;
const int LED_CONTROL_PIN = 17;


void setup() {
  Serial.begin(115200);
  pinMode(PIR_SENSOR_OUTPUT_PIN, INPUT);
  pinMode(LED_CONTROL_PIN, OUTPUT);
  digitalWrite(LED_CONTROL_PIN, LOW);
 
  Serial.println("Dang lam nong cam bien, xin cho 20s");
  delay(20000);
  Serial.println("Cam bien san sang");

}

void loop() {
    int hasMove;
    hasMove = digitalRead(PIR_SENSOR_OUTPUT_PIN);
    if(hasMove == LOW)
    {
      Serial.print("LOW\n");
      digitalWrite(LED_CONTROL_PIN, LOW);

      delay(1000);
    }
    else
    {
      Serial.print("HIGH\n");
      digitalWrite(LED_CONTROL_PIN, HIGH);
      delay(20000);
    }
    delay(100);
}
