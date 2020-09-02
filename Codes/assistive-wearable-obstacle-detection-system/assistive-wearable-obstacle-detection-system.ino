#include "SPI.h"
#include "SD.h"
#include "Adafruit_VS1053.h"
/* --------------------------------------------------------------- */
//uncomment to enable debugging
//#define DEBUG
#include "DebugUtils.h"
/* --------------------------------------------------------------- */
/* Device Operating Configurations */
#ifdef DEBUG
#define BAUD_RATE                       115200
#endif
#define ITERATION_INTERVAL                 300     //in milliseconds, amount of time to wait before next iteration
#define TRIGGER_WAIT_DURATION           60      //in milliseconds, wait before sending another trigger signal
#define MAX_DISTANCE                    50      //maximum distance considered in centimeters
#define PWM_VALUE_PERCENTAGE            70      //0 - 100, maximum output percentage PWM for vibration motors
#define STAIR_ELEV_DISTANCE             20      //in centimeters
#define VIBRATION_DURATION              100     //in milliseconds
#define DEFAULT_VOLUME                  20      //default starting volume, will adjust based on analog reading on volume adjust potentiometer.
#define VOLUME_ADJ_SENSITIVITY          50      //sensitivity of the volume adjust parameter, lower is more sensitive
#if VOLUME_ADJ_SENSITIVITY <= 0
#error "Volume Adjust Sensitivity cannot be less than or equal to zero."
#endif
/* --------------------------------------------------------------- */
/* PIN Configuration */
//    ultrasonic sensor pins
const unsigned short US_TRIG_PIN = 4;
const unsigned short US_ECHO_FRONT_PIN = 5;
const unsigned short US_ECHO_LEFT_PIN = 7;
const unsigned short US_ECHO_RIGHT_PIN = 8;
const unsigned short US_ECHO_STAIRS_PIN = 5;
//    PIR sensor pins
const unsigned short PIR_RX_LEFT_PIN = 7;
const unsigned short PIR_RX_RIGHT_PIN = 8;
//    output pin to drive vibration motors
const unsigned short PWM_PIN_RIGHT = 3;
const unsigned short PWM_PIN_FRONT = 6;
const unsigned short PWM_PIN_LEFT = 10;
//    selector pin for choosing between:
//      0 : ultrasonic sensors for general obstacles and common trigger
//      1 : ultrasonic sensor for stair detection, PIR sensors, and microsd card reader
const unsigned short SELECTOR_PIN = 2;
//    SPI pins common for sd card and vs1053 audio codec
// default MISO_PIN = 12;
// default MOSI_PIN = 11;
// default SCK_PIN = 13;
//    chip enable pin for sd card
const unsigned short SD_CS_PIN = 4;
//    extra pins for vs1053 audio codec
const unsigned short VS1053_DREQ = 9;
const unsigned short VS1053_XDCS = A2;
const unsigned short VS1053_SDCS = A1;
const unsigned short VS1053_XRST = A0;
//    speaker volume adjust
const unsigned short VOLUME_ADJ = A3;
/* --------------------------------------------------------------- */
//    TAKE NOTE OF THE SHARED PINS IN THIS CONFIG
//    D4 - OUTPUT PIN
//    D5 - INPUT PIN
//    D7 - INPUT PIN
//    D8 - INPUT PIN
/* --------------------------------------------------------------- */
unsigned short readUltrasonicState, readMovementState;
bool leftActivePrevious, rightActivePrevious;
bool VS1053CodecFailed, sdCardFailed;
unsigned long millisNow;
unsigned short speaker_volume;
#ifdef DEBUG
unsigned long seqNo;
#endif
/* --------------------------------------------------------------- */
//output variable for stair detection algorithm
long prevStairDistance;
/* --------------------------------------------------------------- */
enum MOVEMENT_PATTERN {NO_MOVEMENT, RIGHTWARD, LEFTWARD};
/* --------------------------------------------------------------- */
Adafruit_VS1053_FilePlayer playback = 
  Adafruit_VS1053_FilePlayer(
    MOSI_PIN, 
    MISO_PIN, 
    SCK_PIN, 
    VS1053_XRST, 
    VS1053_SDCS,
    VS1053_XDCS,
    VS1053_DREQ, 
    SD_CS_PIN);
/* --------------------------------------------------------------- */
void setup() {
  //ultrasonic pins
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_FRONT_PIN, INPUT);
  pinMode(US_ECHO_LEFT_PIN, INPUT);
  pinMode(US_ECHO_RIGHT_PIN, INPUT);
  //pir pins
  pinMode(PIR_RX_RIGHT_PIN, INPUT);
  pinMode(PIR_RX_LEFT_PIN, INPUT);
  //pwm vibration motor pins
  pinMode(PWM_PIN_RIGHT, OUTPUT);
  pinMode(PWM_PIN_FRONT, OUTPUT);
  pinMode(PWM_PIN_LEFT, OUTPUT);

  //other pins
  pinMode(SELECTOR_PIN, OUTPUT);
  pinMode(VOLUME_ADJ, INPUT);

  readMovementState = readUltrasonicState = 0;
  
  selectDeviceLine(1);
  #ifdef DEBUG
  Serial.begin(BAUD_RATE);
  seqNo = 0;
  #endif
  DEBUG_PRINTLN("Obstacle Detection System Debugger");

  VS1053CodecFailed = !playback.begin();
  if (VS1053CodecFailed){
    DEBUG_PRINTLN("ERROR: Couldn't initialize VS1053. Check connections and configuration.");
  } else {
    unsigned int raw_vol = analogRead(VOLUME_ADJ) / VOLUME_ADJ_SENSITIVITY;
    speaker_volume = map(raw_vol, 0, 1023 / VOLUME_ADJ_SENSITIVITY, 0, 255); //lower is louder, 255 maximum value
    playback.setVolume(speaker_volume, speaker_volume);
    playback.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT);
  }

  sdCardFailed = SD.begin(SD_CS_PIN);
  if (sdCardFailed){
    DEBUG_PRINTLN("ERROR: Couldn't initialize connection with SD Card. Check connections and configuration.");
  } 
  #ifdef DEBUG
  else {
    // list files in directory
    DEBUG_PRINTLN("Listing directory for SD Card.");
    printDirectory(SD.open("/"), 0);
  }
  #endif

  if (VS1053CodecFailed || sdCardFailed){
    DEBUG_PRINTLN("WARNING: VS1053 codec or SD Card didn't initialize properly. Speech warnings functionality will cease to function.");
  } else {
    startTone();
  }

  beginVibrationMotors();
}
/* --------------------------------------------------------------- */
void loop() {
  long durationMeasured, distanceMeasuredInCm;
  bool leftActive, rightActive;

  #ifdef DEBUG
  seqNo++;
  DEBUG_PRINT("Sequence #");
  DEBUG_PRINT(seqNo);
  DEBUG_PRINTLN();
  #endif
  
  // motion detection and stair detection
  // will refuse to work if audio is not initialized properly
  if (!sdCardFailed && !VS1053CodecFailed)
  {
    selectDeviceLine(1);
    leftActive = digitalRead(PIR_RX_LEFT_PIN) == HIGH;
    rightActive = digitalRead(PIR_RX_RIGHT_PIN) == HIGH;
    
    switch(readMovementState){
      case 0:
        //record initial position
        leftActivePrevious = leftActive;
        rightActivePrevious = rightActive;
        readMovementState++;
        break;
      case 1:
        //record final position
        enum MOVEMENT_PATTERN pattern;
        pattern = determineMovementPattern(leftActive, rightActive, leftActivePrevious, rightActivePrevious);
        if (pattern == RIGHTWARD)
          playback.startPlayingFile("/filipino_obstacle_right.mp3");
        else if (pattern == LEFTWARD)
          playback.startPlayingFile("/filipino_obstacle_left.mp3");
        //NO_MOVEMENT pattern or values unaccounted for will do nothing
        readMovementState--;
        break;
      default:
        readMovementState = 0;
        DEBUG_PRINTLN("WARNING: read movement pattern state is invalid, resetting to zero.");
    }

    //  consider elevated obstacle detection first before horizontal obstacle
    switch (readUltrasonicState)
    {
      //record initial distance
      case 0:
        //  select line 0 to choose the US sensor devices and common trigger
        selectDeviceLine(0);
        triggerUltrasonicSensor(US_TRIG_PIN);
        //  select line 1 to choose the microsd card reader, PIR sensors, and US sensor for stair detection
        selectDeviceLine(1);

        durationMeasured = pulseIn(US_ECHO_STAIRS_PIN, HIGH);
        prevStairDistance = microsecondsToCentimeters(durationMeasured);
        DEBUG_PRINT("Initial Stair Distance = ");
        DEBUG_PRINT(prevStairDistance);
        DEBUG_PRINT(" cm\n");
        readUltrasonicState++;
        DEBUG_PRINT("Stair State = ");
        DEBUG_PRINT(readUltrasonicState);
        DEBUG_PRINTLN();
        break;
      //record final distance and compare, then clear
      case 1:
        long finalDistanceMeasured;
        //  select line 0 to choose the US sensor devices and common trigger
        selectDeviceLine(0);
        triggerUltrasonicSensor(US_TRIG_PIN);

        //  select line 1 to choose the microsd card reader, PIR sensors, and US sensor for stair detection
        selectDeviceLine(1);

        durationMeasured = pulseIn(US_ECHO_STAIRS_PIN, HIGH);
        finalDistanceMeasured = microsecondsToCentimeters(durationMeasured);
        DEBUG_PRINT("Final Stair Distance = ");
        DEBUG_PRINT(finalDistanceMeasured);
        DEBUG_PRINT(" cm\n");
        //compare if there is a significant change in elevation
        //Republic Act No. 6541 - SECTION 3.01.08 - (h)
        //low elevation - downward stairs
        if (finalDistanceMeasured - prevStairDistance >= STAIR_ELEV_DISTANCE)
          playback.startPlayingFile("/filipino_obstacle_lowElevation.mp3");
        //high elevation - upward stairs
        else if (finalDistanceMeasured - prevStairDistance <= -STAIR_ELEV_DISTANCE)
          playback.startPlayingFile("/filipino_obstacle_elevated.mp3");
        readUltrasonicState--;
        break;
      default:
        readUltrasonicState = 0;
        DEBUG_PRINTLN("WARNING: ultrasonic detect state is invalid, resetting to zero.");
    }
  } else
  {
    DEBUG_PRINTLN("ERROR: Unable to proceed with motion/stair detection, SD Card has failed at setup.");
  }

  //  select line 0 to choose the US sensor devices and common trigger
  selectDeviceLine(0);
  
  millisNow = millis();
  // account for possibility of overlapping triggers from vertical ultrasonic trigger
  while(millis() < millisNow + TRIGGER_WAIT_DURATION);
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  // a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  triggerUltrasonicSensor(US_TRIG_PIN);
  DEBUG_PRINTLN("Front Ultrasonic triggered.");
  durationMeasured = pulseIn(US_ECHO_FRONT_PIN, HIGH);
  distanceMeasuredInCm = microsecondsToCentimeters(durationMeasured);
  DEBUG_PRINT("Front Ultrasonic Distance = ");
  DEBUG_PRINT(distanceMeasuredInCm);
  DEBUG_PRINT(" cm\n");
  driveMotor(distanceMeasuredInCm, PWM_PIN_FRONT, VIBRATION_DURATION);
  millisNow = millis();
  // account for possibility of overlapping triggers
  while(millis() < millisNow + TRIGGER_WAIT_DURATION);

  triggerUltrasonicSensor(US_TRIG_PIN);
  DEBUG_PRINTLN("Left Ultrasonic triggered.");
  durationMeasured = pulseIn(US_ECHO_LEFT_PIN, HIGH);
  distanceMeasuredInCm = microsecondsToCentimeters(durationMeasured);
  DEBUG_PRINT("Left Ultrasonic Distance = ");
  DEBUG_PRINT(distanceMeasuredInCm);
  DEBUG_PRINT(" cm\n");
  driveMotor(distanceMeasuredInCm, PWM_PIN_LEFT, VIBRATION_DURATION);
  millisNow = millis();
  // account for possibility of overlapping triggers
  while(millis() < millisNow + TRIGGER_WAIT_DURATION);

  triggerUltrasonicSensor(US_TRIG_PIN);
  DEBUG_PRINTLN("Right Ultrasonic triggered.");
  durationMeasured = pulseIn(US_ECHO_RIGHT_PIN, HIGH);
  distanceMeasuredInCm = microsecondsToCentimeters(durationMeasured);
  DEBUG_PRINT("Right Ultrasonic Distance = ");
  DEBUG_PRINT(distanceMeasuredInCm);
  DEBUG_PRINT(" cm\n");
  driveMotor(distanceMeasuredInCm, PWM_PIN_RIGHT, VIBRATION_DURATION);

  turnOffMotor(PWM_PIN_FRONT);
  turnOffMotor(PWM_PIN_LEFT);
  turnOffMotor(PWM_PIN_RIGHT);

  delay(ITERATION_INTERVAL);
}
/* --------------------------------------------------------------- */
long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is roughly 340 m/s or 29 microseconds per centimeter.
  // This does not take into account the temperature of the surroundings.
  // The actual formula is d = 334 + 0.6*T where T is the ambient temperature
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 58;
}
/* --------------------------------------------------------------- */
void triggerUltrasonicSensor(unsigned int TRIGGER_PIN)
{
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
}
/* --------------------------------------------------------------- */
void driveMotor(long distanceMeasured, unsigned int motor_pin, unsigned int vibrationDuration)
{
  unsigned int pwmVal;
  if (distanceMeasured > MAX_DISTANCE || distanceMeasured <= 0) return;
  pwmVal =  map(distanceMeasured, 0, MAX_DISTANCE, 255 * PWM_VALUE_PERCENTAGE / 100, 0);
  
  DEBUG_PRINT("PWM Value: ");
  DEBUG_PRINT(pwmVal);
  DEBUG_PRINT(", driving motor for pin = ");
  DEBUG_PRINT(motor_pin);
  DEBUG_PRINTLN();
  
  analogWrite(motor_pin, pwmVal);
  delay(vibrationDuration);
}
/* --------------------------------------------------------------- */
void turnOffMotor(unsigned int motor_pin)
{
  analogWrite(motor_pin, 0);
}
/* --------------------------------------------------------------- */
void beginVibrationMotors()
{
  for (unsigned int i = 0; i < 3; i++){
    driveMotor(1, PWM_PIN_RIGHT, VIBRATION_DURATION);
    driveMotor(1, PWM_PIN_FRONT, VIBRATION_DURATION);
    driveMotor(1, PWM_PIN_LEFT, VIBRATION_DURATION);
    delay(100);
    turnOffMotor(PWM_PIN_FRONT);
    turnOffMotor(PWM_PIN_LEFT);
    turnOffMotor(PWM_PIN_RIGHT);
  }
}
/* --------------------------------------------------------------- */
void startTone()
{
  playback.playFullFile("/startup.mp3");
}
/* --------------------------------------------------------------- */
void selectDeviceLine(unsigned int lineNumber)
{
  switch (lineNumber)
  {
    case 0:
      // select line 0 to choose the US sensor devices and common trigger
      digitalWrite(SELECTOR_PIN, LOW);
      break;
    case 1:
      // select line 1 to choose the microsd card reader, PIR sensors, and US sensor for stair detection
      digitalWrite(SELECTOR_PIN, HIGH);
      break;
    default:
      digitalWrite(SELECTOR_PIN, HIGH);
  }
}
/* --------------------------------------------------------------- */
enum MOVEMENT_PATTERN determineMovementPattern(bool leftCurrent, bool rightCurrent, bool leftPrev, bool rightPrev){
  if (leftPrev && leftCurrent && rightPrev && rightCurrent)
    return NO_MOVEMENT;
  else if (leftPrev && rightCurrent){
    return RIGHTWARD;
  }
  else if (rightPrev && leftCurrent){
    return LEFTWARD;
  }
  else {
    return NO_MOVEMENT;  
  }
}
/* --------------------------------------------------------------- */
/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       DEBUG_PRINT('\t');
     }
     DEBUG_PRINTLN(entry.name());
     if (entry.isDirectory()) {
       DEBUG_PRINTLN("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       DEBUG_PRINT("\t\t");
       DEBUG_PRINTLN(entry.size(), DEC);
     }
     entry.close();
   }
}
/* --------------------------------------------------------------- */
