#include <SPI.h>
#include <SD.h>
#include <TMRpcm.h>
#include <pcmRF.h>
#include "DebugUtils.h"
/* --------------------------------------------------------------- */
//#define DEBUG
/* --------------------------------------------------------------- */
/* Device Operating Configurations */
#define SENSOR_INTERVAL               500     //in milliseconds, amount of time to wait before next measurement
#define MAX_DISTANCE                  50      //in centimeters
#define STAIR_ELEV_DISTANCE           20      //in centimeters
#define VIBRATION_DURATION            500     //in milliseconds
#define SPEAKER_VOLUME                6       //set volume to 6 (loudest 7, no sound 0) might be distorted if too loud
#define SPEAKER_SUPERSAMPLING_ENABLED 0       //enable 2x supersampling
#define SPEAKER_AUDIO_LOOP_ENABLED    0       //prevent repeated playing of same file
/* --------------------------------------------------------------- */
#define OBSTACLE_LVL1_DISTANCE MAX_DISTANCE / 4
#define OBSTACLE_LVL2_DISTANCE MAX_DISTANCE / 2
#define OBSTACLE_LVL3_DISTANCE MAX_DISTANCE * 3 / 4
#define OBSTACLE_LVL4_DISTANCE MAX_DISTANCE
#define PWM_LVL1_VALUE 255
#define PWM_LVL2_VALUE 255 * 3 / 4
#define PWM_LVL3_VALUE 255 / 2
#define PWM_LVL4_VALUE 255 / 4
#define PWM_LVL5_VALUE 0
/* --------------------------------------------------------------- */
#if SPEAKER_VOLUME > 7 || SPEAKER_VOLUME < 0
#undef SPEAKER_VOLUME
#define SPEAKER_VOLUME 6
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
//      0 : ultrasonic sensors for general obstacles
//      1 : ultrasonic sensor for stair detection, PIR sensors, and microsd card reader
const unsigned short SELECTOR_PIN = 2;
//    microsd card reader and speaker pins for audio playback
const unsigned short SD_MISO_PIN = 12;
const unsigned short SD_MOSI_PIN = 11;
const unsigned short SD_SCK_PIN = 13;
const unsigned short SD_CS_PIN = 4;
const unsigned short SPEAKER_PIN = 9;
/* --------------------------------------------------------------- */
//    TAKE NOTE OF THE SHARED PINS IN THIS CONFIG
//    D4 - OUTPUT PIN
//    D5 - INPUT PIN
//    D7 - INPUT PIN
//    D8 - INPUT PIN
/* --------------------------------------------------------------- */
unsigned short stairDetectState;
bool sdCardFailed;
#ifdef DEBUG
unsigned long seqNo;
#endif
/* --------------------------------------------------------------- */
//output variable for stair detection algorithm
long prevStairDistance;
/* --------------------------------------------------------------- */
TMRpcm sdAudioPlayer;
/* --------------------------------------------------------------- */
void setup() {
  // set the pin modes of the used pins
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_FRONT_PIN, INPUT);
  pinMode(US_ECHO_LEFT_PIN, INPUT);
  pinMode(US_ECHO_RIGHT_PIN, INPUT);

  pinMode(PIR_RX_RIGHT_PIN, INPUT);
  pinMode(PIR_RX_LEFT_PIN, INPUT);

  pinMode(PWM_PIN_RIGHT, OUTPUT);
  pinMode(PWM_PIN_FRONT, OUTPUT);
  pinMode(PWM_PIN_LEFT, OUTPUT);
  pinMode(SELECTOR_PIN, OUTPUT);
  stairDetectState = 0;
  selectDeviceLine(1);
  #ifdef DEBUG
  Serial.begin(9600);
  seqNo = 0;
  #endif
  if (initializeSDCardReader())
  {
    // sd card communication is functional
    DEBUG_PRINTLN("SD Card Reader initialized successfully.");
    sdAudioPlayer.speakerPin = SPEAKER_PIN;
    sdAudioPlayer.setVolume(SPEAKER_VOLUME);
    sdAudioPlayer.quality(SPEAKER_SUPERSAMPLING_ENABLED);
    sdAudioPlayer.loop(SPEAKER_AUDIO_LOOP_ENABLED);
    DEBUG_PRINTLN("Audio Playback Settings initialized.");
    endSDCardReader();
    sdCardFailed = false;
  } else 
  {
    DEBUG_PRINTLN("ERROR: SD Card Reader failed to initialize.");
    sdCardFailed = true;
  }

  beginVibrationMotors();
  startTone();
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
  if (!sdCardFailed)
  {
    selectDeviceLine(1);

    // conside moving obstacles first
    leftActive = digitalRead(PIR_RX_LEFT_PIN) == HIGH;
    rightActive = digitalRead(PIR_RX_RIGHT_PIN) == HIGH;

    if (leftActive && rightActive)
    {
      //enter only when sd card reader connection is successful
      if (initializeSDCardReader())
      {
        //  select and play obstacle_front.wav from sd card
        playFile("ofron.wav");
        while (sdAudioPlayer.isPlaying()); //wait for audio playback to finish
        endSDCardReader();
      } else
      {
        DEBUG_PRINTLN("ERROR: Failed to play audio playback for front obstacle detection.");
      }
    }
    else if (leftActive)
    {
      //enter only when sd card reader connection is successful
      if (initializeSDCardReader())
      {
        //  select and play obstacle_left.wav from sd card
        playFile("oleft.wav");
        while (sdAudioPlayer.isPlaying()); //wait for audio playback to finish
        endSDCardReader();
      } else
      {
        DEBUG_PRINTLN("ERROR: Failed to play audio playback for left obstacle detection.");
      }
    }
    else if (rightActive)
    {
      //enter only when sd card reader connection is successful
      if (initializeSDCardReader())
      {
        //  select and play obstacle_right.wav from sd card
        playFile("orght.wav");
        while (sdAudioPlayer.isPlaying()); //wait for audio playback to finish
        endSDCardReader();
      } else
      {
        DEBUG_PRINTLN("ERROR: Failed to play audio playback for right obstacle detection.");
      }
    }

    //  consider stair detection case
    switch (stairDetectState)
    {
      //starting point, ignore for first run
      case 0:
        stairDetectState++;
        DEBUG_PRINT("Stair State = ");
        DEBUG_PRINT(stairDetectState);
        DEBUG_PRINTLN();
        break;
      //record initial distance
      case 1:
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
        stairDetectState++;
        DEBUG_PRINT("Stair State = ");
        DEBUG_PRINT(stairDetectState);
        DEBUG_PRINTLN();
        break;
      //record final distance and compare, then clear
      case 2:
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
        {
          DEBUG_PRINTLN("Low Elevation Stairs detected.");
          //enter only when sd card reader connection is successful
          if (initializeSDCardReader())
          {
            //  select and play obstacle_lowElevation.wav from sd card
            playFile("olwlv.wav");
            while (sdAudioPlayer.isPlaying()); //wait for audio playback to finish
            endSDCardReader();
          } else
          {
            DEBUG_PRINTLN("ERROR: Failed to play audio playback for low elevation obstacle.");
          }
        }
        //high elevation - upward stairs
        else if (finalDistanceMeasured - prevStairDistance <= -STAIR_ELEV_DISTANCE)
        {
          DEBUG_PRINTLN("High Elevation Stairs detected.");
          //enter only when sd card reader connection is successful
          if (initializeSDCardReader())
          {
            //  select and play obstacle_elevated.wav from sd card
            playFile("oelev.wav");
            while (sdAudioPlayer.isPlaying()); //wait for audio playback to finish
            endSDCardReader();
          } else
          {
            DEBUG_PRINTLN("ERROR: Failed to play audio playback for high elevation obstacle.");
          }
        }
        stairDetectState--;
        break;
    }
  } else
  {
    DEBUG_PRINTLN("ERROR: Unable to proceed with motion/stair detection, SD Card has failed at setup.");
  }

  //  select line 0 to choose the US sensor devices and common trigger
  selectDeviceLine(0);

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  // a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  triggerUltrasonicSensor(US_TRIG_PIN);
  DEBUG_PRINTLN("Front Ultrasonic triggered.");
  durationMeasured = pulseIn(US_ECHO_FRONT_PIN, HIGH);
  distanceMeasuredInCm = microsecondsToCentimeters(durationMeasured);
  driveMotor(distanceMeasuredInCm, true, true, VIBRATION_DURATION);

  triggerUltrasonicSensor(US_TRIG_PIN);
  DEBUG_PRINTLN("Left Ultrasonic triggered.");
  durationMeasured = pulseIn(US_ECHO_LEFT_PIN, HIGH);
  distanceMeasuredInCm = microsecondsToCentimeters(durationMeasured);
  driveMotor(distanceMeasuredInCm, true, false, VIBRATION_DURATION);

  triggerUltrasonicSensor(US_TRIG_PIN);
  DEBUG_PRINTLN("Right Ultrasonic triggered.");
  durationMeasured = pulseIn(US_ECHO_RIGHT_PIN, HIGH);
  distanceMeasuredInCm = microsecondsToCentimeters(durationMeasured);
  driveMotor(distanceMeasuredInCm, false, true, VIBRATION_DURATION);

  turnOffMotor(PWM_PIN_FRONT);
  turnOffMotor(PWM_PIN_LEFT);
  turnOffMotor(PWM_PIN_RIGHT);

  delay(SENSOR_INTERVAL);
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
//  4 levels
//  Level 1(Highest)         - measured <= MAX_DISTANCE / 4
//  Level 2                  - measured <= MAX_DISTANCE / 2
//  Level 3                  - measured <= MAX_DISTANCE * 3/4
//  Level 4                  - measured <= MAX_DISTANCE
//  Level 5(Not an obstacle) - measured > MAX_DISTANCE
int determineObstacleLevel(long distanceInCm)
{
  if      (distanceInCm <= OBSTACLE_LVL1_DISTANCE) return 1;
  else if (distanceInCm <= OBSTACLE_LVL2_DISTANCE) return 2;
  else if (distanceInCm <= OBSTACLE_LVL3_DISTANCE) return 3;
  else if (distanceInCm <= OBSTACLE_LVL4_DISTANCE) return 4;
  else return 5;
}
/* --------------------------------------------------------------- */
int determinePWMLevel(int obstacleLevel)
{
  switch (obstacleLevel)
  {
    case 4:
      return PWM_LVL4_VALUE;
    case 3:
      return PWM_LVL3_VALUE;
    case 2:
      return PWM_LVL2_VALUE;
    case 1:
      return PWM_LVL1_VALUE;
    case 5:
    default:
      return PWM_LVL5_VALUE;
  }
}
/* --------------------------------------------------------------- */
void triggerUltrasonicSensor(int TRIGGER_PIN)
{
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN, LOW);
}
/* --------------------------------------------------------------- */
void driveMotor(long distanceMeasured, bool enableLeft, bool enableRight, int vibrationDuration)
{
  int obsLevel, pwmVal;

  obsLevel = determineObstacleLevel(distanceMeasured);
  pwmVal =  determinePWMLevel(obsLevel);
  DEBUG_PRINT("Obstacle Level: ");
  DEBUG_PRINT(obsLevel);
  DEBUG_PRINTLN();
  DEBUG_PRINT("PWM Value: ");
  DEBUG_PRINT(pwmVal);
  DEBUG_PRINTLN();
  if (obsLevel != 5)
  {
    if (enableLeft && enableRight)
    {
      analogWrite(PWM_PIN_FRONT, pwmVal);
      delay(vibrationDuration);
    }
    else if (enableLeft && !enableRight)
    {
      digitalWrite(PWM_PIN_LEFT, HIGH);
      delay(vibrationDuration);
    }
    else if (!enableLeft && enableRight)
    {
      analogWrite(PWM_PIN_RIGHT, pwmVal);
      delay(vibrationDuration);
    }
  }
}
/* --------------------------------------------------------------- */
void turnOffMotor(int motor_pin)
{
  analogWrite(motor_pin, 0);
}
/* --------------------------------------------------------------- */
void beginVibrationMotors()
{
  driveMotor(1, true, true, VIBRATION_DURATION);
  driveMotor(1, true, false, VIBRATION_DURATION);
  driveMotor(1, false, true, VIBRATION_DURATION);
  delay(100);
  turnOffMotor(PWM_PIN_FRONT);
  turnOffMotor(PWM_PIN_LEFT);
  turnOffMotor(PWM_PIN_RIGHT);
  delay(100);
  driveMotor(1, true, true, VIBRATION_DURATION);
  driveMotor(1, true, false, VIBRATION_DURATION);
  driveMotor(1, false, true, VIBRATION_DURATION);
  delay(100);
  turnOffMotor(PWM_PIN_FRONT);
  turnOffMotor(PWM_PIN_LEFT);
  turnOffMotor(PWM_PIN_RIGHT);
  delay(100);
  driveMotor(1, true, true, VIBRATION_DURATION);
  driveMotor(1, true, false, VIBRATION_DURATION);
  driveMotor(1, false, true, VIBRATION_DURATION);
  delay(100);
  turnOffMotor(PWM_PIN_FRONT);
  turnOffMotor(PWM_PIN_LEFT);
  turnOffMotor(PWM_PIN_RIGHT);
}
/* --------------------------------------------------------------- */
void playFile(String fileName)
{
  sdAudioPlayer.stopPlayback();

  int strLen = fileName.length() + 1; // include also the string terminator for the char count
  char charFileName[strLen];
  fileName.toCharArray(charFileName, strLen);
  sdAudioPlayer.play(charFileName);
}
/* --------------------------------------------------------------- */
void errorTone()
{
  tone(SPEAKER_PIN, 1000); // Send 1KHz sound signal...
  delay(150);
  noTone(SPEAKER_PIN);
  tone(SPEAKER_PIN, 1000); // Send 1KHz sound signal...
  delay(150);
  noTone(SPEAKER_PIN);
  tone(SPEAKER_PIN, 1000); // Send 1KHz sound signal...
  delay(150);
  noTone(SPEAKER_PIN);
}
/* --------------------------------------------------------------- */
void startTone()
{
  tone(SPEAKER_PIN, 500);
  delay(150);
  noTone(SPEAKER_PIN);
  tone(SPEAKER_PIN, 750);
  delay(150);
  noTone(SPEAKER_PIN);
  tone(SPEAKER_PIN, 1000);
  delay(150);
  noTone(SPEAKER_PIN);
}
/* --------------------------------------------------------------- */
bool initializeSDCardReader()
{
  if (!SD.begin(SD_CS_PIN))
  {
    // if error, notify with tone, but proceed with rest of program, audio playback will not function
    errorTone();
    return false;
  }
  else return true;
}
/* --------------------------------------------------------------- */
void endSDCardReader()
{
  SPI.end();
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
