/setup method for each sensor
//loop method for each sensor
int led = 13;           
int button;
int buzzer = 50;
void setupButton(int pinNumber){
  button = pinNumber;
  pinMode(pinNumber,INPUT);
}
void checkSensor(int sensor, int actuator){
  if(digitalRead(sensor) == HIGH){
    digitalWrite(actuator, HIGH);
  }
  else{
    digitalWrite(actuator, LOW);
  }
}
void setup() {
  pinMode(led,OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  setupButton(51);
}
 
void loop() {
  checkSensor(button, buzzer);
  checkSensor(button, led);
}
