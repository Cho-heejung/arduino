#define PIN_LED 7
unsigned int a;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  a = 1;
}

void loop() {
  while (a == 1) {
    digitalWrite(PIN_LED, 0);
    delay(1000);
    a = 0;
  }
  for (int i=1;i<=5;i++) {
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
  }
  while(1) {
    digitalWrite(PIN_LED, 1);
  }
}
