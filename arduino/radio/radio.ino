void setup() {
  Serial.begin(9600);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
}

void loop() {
  int a = digitalRead(5);

  Serial.println(a);
  int delay = 10;
}
