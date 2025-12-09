void setup() {
  pinMode(10, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(5, INPUT);

}

void loop() {
  if(digitalRead(5)){
    analogWrite(10,55);
    analogWrite(9, 55);
  }
  if(!digitalRead(5)){
    analogWrite(10, 0);
    analogWrite(9, 0);
  }
  // if(digitalRead(6)){
  //   digitalWrite(8, 1);
  // }
  // if(!digitalRead(6)){
  //   digitalWrite(8, 0);
  // }
}
