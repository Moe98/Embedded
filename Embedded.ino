#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include "SoftwareSerial.h"
#include <Arduino_FreeRTOS.h>
#include "semphr.h"

#define buzzer 30
#define alarmLed 31
#define headlights 32

#define lineTracker 33

#define tempSensor A0
#define ldr A1
#define gearStick A2

#define buttonNext 40
#define buttonPause 41
#define buttonPrevious 42

#define ENA 8
#define N1 34
#define N2 35
#define ENB 9
#define N3 36
#define N4 37

#define Start_Byte 0x7E
#define Version_Byte 0xFF
#define Command_Length 0x06
#define End_Byte 0xEF
#define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]

# define ACTIVATED LOW

enum Gear {
  P,
  R,
  N,
  D
};

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
SoftwareSerial mySerial(10, 11);
RTC_DS1307 rtc;
DateTime currentTime;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
double temp;
int gearVal;
Gear gear = P;
bool gearShifted = false;
int night = LOW;
int light;
char state = 0;
int power = 100;
int offset = 0;
int active = HIGH;
int inactive = LOW;

boolean MyisPlaying = false;
SemaphoreHandle_t isPlaying;
SemaphoreHandle_t AccessLCD;
/**
  Sets up a pin connected to a sensor as input
  @param pinNumber connected to the sensor
*/

void setupSensor(int pinNumber) {
  pinMode(pinNumber, INPUT);
}

/**
  Sets up a pin connected to an actuator as output
  @param pinNumber connected to the actuator
*/

void setupActuator(int pinNumber) {
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
bool between(int val, int lowerBound, int upperBound) {
  return val >= lowerBound && val <= upperBound;
}

double analogReadAdapted(int sensor, double RESOLUTION, double STEP) {
  int aref;
  float val;
  aref = analogRead(sensor);
  val = (aref * RESOLUTION);
  val = (val / STEP);
  return val;
}

void readDigitalActDigital(int sensor, int actuator, bool match) {
  if (digitalRead(sensor) == HIGH && match) {
    digitalWrite(actuator, HIGH);
  }
  else {
    digitalWrite(actuator, LOW);
  }
}

void writeLCD(char *string, int column, int row) {
  lcd.setCursor(column, row);
  lcd.print(string);
  //  Serial.println(string);
  //  lcd.setCursor(0, 0);
}

void writeLCD(double val, int column, int row) {
  lcd.setCursor(column, row);
  lcd.print(val);
  //  Serial.println(val);
  //  lcd.setCursor(0, 0);
}

void writeLCD(int val, int column, int row) {
  lcd.setCursor(column, row);
  lcd.print(val, DEC);
  //  Serial.println(val, DEC);
  //  lcd.setCursor(0, 0);
}

void writeLCD(uint16_t val, int column, int row) {
  lcd.setCursor(column, row);
  lcd.print(val, DEC);
  //  Serial.println(val, DEC);
  //  lcd.setCursor(0, 0);
}

void setPower() {
  int pa = power + (offset <= 0 ? offset : 0);
  int pb = power - (offset >= 0 ? offset : 0);

  if (pa > 255) {
    pa = 255;
  }
  else if (pa < 0) {
    pa = 0;
  }

  if (pb > 255) {
    pb = 255;
  }
  else if (pb < 0) {
    pb = 0;
  }

  analogWrite(ENA, pa);
  analogWrite(ENB, pb);
}

void driveMotors() {
  if (gear == D) {
    active = HIGH;
    inactive = LOW;
  }
  else if (gear == R) {
    active = LOW;
    inactive = HIGH;
  }
  else {
    active = LOW;
    inactive = LOW;
  }

  if (Serial.available() > 0) { // Checks whether data is comming from the serial port
    state = Serial.read(); // Reads the data from the serial port
  }

  if (state == 'r') {
    power = power >= 255 ? 255 : power + 1;
    Serial.write(power);
    setPower();
  }
  else if (state == 'f') {
    power = power <= 0 ? 0 : power - 1;
    Serial.write(power);
    setPower();
  }

  else if (state == 'q') {
    offset = offset <= -power ? -power : offset - 1;
    Serial.write(offset);
    setPower();
  }
  else if (state == 'e') {
    offset = offset >= power ? power : offset + 1;
    Serial.write(offset);
    setPower();
  }

  else {
    if (state == 'a') {
      digitalWrite(N1, LOW);
      digitalWrite(N2, LOW);
      digitalWrite(N3, active);
      digitalWrite(N4, inactive);
    }
    else if (state == 'd') {
      digitalWrite(N1, active);
      digitalWrite(N2, inactive);
      digitalWrite(N3, LOW);
      digitalWrite(N4, LOW);
    }
    else if (state == 'w') {
      digitalWrite(N1, active);
      digitalWrite(N2, inactive);
      digitalWrite(N3, active);
      digitalWrite(N4, inactive);
    }
    else if (state == 's') {
      //    else {
      digitalWrite(N1, LOW);
      digitalWrite(N2, LOW);
      digitalWrite(N3, LOW);
      digitalWrite(N4, LOW);
    }
  }
  state = 0;
}

void playFirst()
{
  execute_CMD(0x3F, 0, 0);
  delay(500);
  setVolume(20);
  delay(500);
  execute_CMD(0x11, 0, 1);
  delay(500);
}

void pause()
{
  execute_CMD(0x0E, 0, 0);
  delay(500);
}

void play()
{
  execute_CMD(0x0D, 0, 1);
  delay(500);
}

void playNext()
{
  execute_CMD(0x01, 0, 1);
  delay(500);
}

void playPrevious()
{
  execute_CMD(0x02, 0, 1);
  delay(500);
}

void setVolume(int volume)
{
  execute_CMD(0x06, 0, volume); // Set the volume (0x00~0x30)
  delay(2000);
}

void execute_CMD(byte CMD, byte Par1, byte Par2)
// Excecute the command and parameters
{
  // Calculate the checksum (2 bytes)
  word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
  // Build the command line
  byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge,
                            Par1, Par2, highByte(checksum), lowByte(checksum), End_Byte
                          };
  //Send the command line to the module
  for (byte k = 0; k < 10; k++)
  {
    mySerial.write( Command_line[k]);
  }
}

void setup() {
  //  lineTracking(NULL);

  Serial.begin(9600);
  //Serial.println("Starting up");

  lcd.begin(16, 2);

  if (! rtc.begin()) {
    //Serial.println("Couldn't find RTC");
    while (1);
  }
//  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if (!rtc.isrunning()) {
    //    Serial.println("RTC lost power, lets set the time!");

    // Comment out below lines once you set the date & time.
    // Following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // Following line sets the RTC with an explicit date & time
    // for example to set January 27 2017 at 12:56 you would call:
    // rtc.adjust(DateTime(2017, 1, 27, 12, 56, 0));
  }

  while (1) {
    isPlaying = xSemaphoreCreateBinary();
    if (isPlaying != NULL) {
      break;
    }
    //    Serial.println("Failed to initalize the semaphore");
  }
  xSemaphoreGive(isPlaying);

  while (1) {
    AccessLCD = xSemaphoreCreateMutex();
    if (AccessLCD != NULL) {
      break;
    }
    //    Serial.println("Failed to initalize the semaphore");
  }
  setupActuator(buzzer);
  setupActuator(alarmLed);
  setupActuator(headlights);

  setupSensor(lineTracker);

  setupSensor(tempSensor);
  setupSensor(ldr);
  setupSensor(gearStick);

  pinMode(buttonPause, INPUT);
  digitalWrite(buttonPause, HIGH);
  pinMode(buttonNext, INPUT);
  digitalWrite(buttonNext, HIGH);
  pinMode(buttonPrevious, INPUT);
  digitalWrite(buttonPrevious, HIGH);

  setupActuator(ENA);
  setupActuator(ENB);
  setupActuator(N1);
  setupActuator(N2);
  setupActuator(N3);
  setupActuator(N4);

  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(N1, LOW);
  digitalWrite(N2, LOW);
  digitalWrite(N3, LOW);
  digitalWrite(N4, LOW);

  mySerial.begin (9600);
  delay(1000);
  playFirst();

  xTaskCreate(Driving, "Moving and line tracking", 128, NULL, 5, NULL);
  xTaskCreate(SoundSystem, "Playing MP3 and pausing", 128, NULL, 3, NULL);
  xTaskCreate(SongSwitching, "playing previous/next song on MP3", 128, NULL, 3, NULL);
  xTaskCreate(DisplayLCD, "Write on LCD", 128, NULL, 4, NULL);
  xTaskCreate(SensorReading, "Read temerature, LDR and RTC sensores", 128, NULL, 4, NULL);

  vTaskStartScheduler();
  for ( ;; );
}

void loop() {}

void Driving (void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    driveMotors();
    if (digitalRead(lineTracker) == HIGH) {
      digitalWrite(alarmLed, HIGH);
      digitalWrite(buzzer, HIGH);
    }
    else {
      digitalWrite(alarmLed, LOW);
      digitalWrite(buzzer, LOW);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
}

void SoundSystem(void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {

    //    Serial.println("pause");
    //    Serial.println(digitalRead(buttonPause));
    if (digitalRead(buttonPause) == ACTIVATED)
    {
      //      Serial.println("start");
      //      Serial.println(uxSemaphoreGetCount(isPlaying));
      if (uxSemaphoreGetCount(isPlaying))
      {
        pause();
        xSemaphoreTake(isPlaying, portMAX_DELAY);
        //        Serial.println("take");
        //        Serial.println(uxSemaphoreGetCount(isPlaying));
        //        isPlaying = false;
      } else
      {
        play();
        //        Serial.println("play");
        //        Serial.println(uxSemaphoreGetCount(isPlaying));
        if (!uxSemaphoreGetCount(isPlaying)) {
          xSemaphoreGive(isPlaying);
          //          Serial.println("give");
          //          Serial.println(uxSemaphoreGetCount(isPlaying));
        }
      }
    }


    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
}

void SongSwitching (void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    xSemaphoreTake(isPlaying, portMAX_DELAY);
    if (digitalRead(buttonNext) == ACTIVATED)
    {
      if (isPlaying)
      {
        playNext();
      }
    }

    if (digitalRead(buttonPrevious) == ACTIVATED)
    {
      if (isPlaying)
      {
        playPrevious();
      }
    }
    xSemaphoreGive(isPlaying);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
}

void DisplayLCD (void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    xSemaphoreTake(AccessLCD, portMAX_DELAY);
    lcd.clear();
    writeLCD(temp, 0, 0);
    writeLCD("C", 6, 0);
    switch (gear) {
      case P: writeLCD("P", 13, 0); break;
      case R: writeLCD("R", 13, 0); break;
      case N: writeLCD("N", 13, 0); break;
      case D: writeLCD("D", 13, 0); break;
      default: writeLCD("?", 13, 0); break;
    }

    writeLCD(currentTime.day(), 0, 1);
    writeLCD("/", 2, 1);
    writeLCD(currentTime.month(), 3, 1);
    writeLCD("/", 5, 1);
    writeLCD(currentTime.year(), 6, 1);

    writeLCD(currentTime.hour(), 11, 1);
    writeLCD(":", 13, 1);
    writeLCD(currentTime.minute(), 14, 1);

    xSemaphoreGive(AccessLCD);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
  }
}

void SensorReading (void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    xSemaphoreTake(AccessLCD, portMAX_DELAY);

    temp = analogReadAdapted(tempSensor, 4.88, 10);

    gearVal = analogRead(gearStick);
    if (gearShifted) {
      gearShifted = !between(gearVal, 300, 700);
    }
    else if (gearVal < 300) {
      gearShifted = true;
      gear = gear == P ? P : gear - 1;
    }
    else if (700 < gearVal) {
      gearShifted = true;
      gear = gear == D ? D : gear + 1;
    }

    light = analogRead(ldr);
    if (light < 300) {
      night = HIGH;
    }
    else if (700 < light) {
      night = LOW;
    }
    digitalWrite(headlights, night);

    currentTime = rtc.now();

    xSemaphoreGive(AccessLCD);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
}
