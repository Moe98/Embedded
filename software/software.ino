#define led 13
#define button 51
#define buzzer 50
#define tempSensor A0
#define NOTSET -1
#define DISCRETE 1
#define ANALOG 2

/**
  Sets up a pin connected to a sensor as input
  @param pinNumber connected to the sensor
*/

void setupSensor(int pinNumber){
  pinMode(pinNumber, INPUT);
}

/**
  Sets up a pin connected to an actuator as output
  @param pinNumber connected to the actuator
*/

void setupActuator(int pinNumber){
  pinMode(pinNumber, OUTPUT);
}

/**
  If given both the sensor and an actuator it applies the change of the sensor on the actuator. If given a sensor only then it returns its reading.

  @param sensor pin number of sensor.
  @param actuator pin number of actuator.
  @param RESOLUTION resolution of the sensor, if analog.
  @param STEP output of volts/Unit.
  @param type determines whether the sensor is analog or discrete
  @return if actuator isn't passed then the reading of the sensor otherwise 0.
*/

double checkSensor(int sensor, int actuator, double RESOLUTION, double STEP, int type){
  if(actuator == NOTSET){
    if(type == ANALOG){
      int aref;
      float val;
      aref = analogRead(sensor);
      val = (aref * RESOLUTION);
      val = (val / STEP);
      return val; 
    }
    if(type == DISCRETE){
      //do stuff
    }
  }
  else{
    if(digitalRead(sensor) == HIGH){
      digitalWrite(actuator, HIGH);
    }
    else{
      digitalWrite(actuator, LOW);
    }
    return 0;
  }
}

void setup() {
  Serial.begin(9600);
  setupActuator(led);
  setupActuator(buzzer);
  //pinMode(led,OUTPUT);
  //pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  setupSensor(button);
  setupSensor(tempSensor);
}
 
void loop() {
  /**
  float temp;
  temp = checkSensor(tempSensor, NOTSET, 4.88, 10, ANALOG);
  Serial.print(temp);
  Serial.println("°C\n");
  */
  checkSensor(button, buzzer, NOTSET, NOTSET, NOTSET);
  checkSensor(button, led, NOTSET, NOTSET, NOTSET);
}
