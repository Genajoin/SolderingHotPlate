#include <Arduino.h>
#include "LiquidCrystal.h" // https://github.com/arduino-libraries/LiquidCrystal
#include <Thermistor.h>
#include <NTC_Thermistor.h> // https://github.com/YuriiSalimov/NTC_Thermistor
#include <SmoothThermistor.h>

#define SENSOR_PIN A0
#define REFERENCE_RESISTANCE 4700
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950
#define SMOOTHING_FACTOR 5

#define CONTRAST_PIN 3
#define LCD_LED_PIN 9
#define BUZZER_PIN 5 // PD5
#define RELAY_PIN 8  // PB0
#define KEY_OK_PIN 10
#define KEY_PLUS_PIN 15
#define KEY_MINUS_PIN 16
#define KEY_BACK_PIN 17

#define KEY_TIMEOUT 200
#define LCD_TIMEOUT 500

enum PlateMode
{
  OFF = 0,
  MODE0,
  MODE1
};

#ifdef DEBUG
PlateMode plateMode = MODE0;
float Setpoint = 300.0;
#else
PlateMode plateMode = OFF;
float Setpoint = 150.0;
#endif
int contrast = 116;  // init pwm [0;255]
int ledBright = 100; // init pwm [0;255]
const char array1[] = "HeatPlateAlpisto";
unsigned long msNow;
unsigned long windowStartTime, nextSwitchTime, nextLcdTime, nextKeyTime;
float Input, Output;
bool relayStatus = false;
LiquidCrystal lcd(12, 11, 2, 4, 6, 7);
Thermistor *thermistor;

void toneOn()
{
#ifndef DEBUG
  analogWrite(BUZZER_PIN, 128);
#endif
}
void toneOff()
{
  analogWrite(BUZZER_PIN, 0);
}

void toneKeyPress()
{
  toneOn();
  delay(100);
  toneOff();
}

void RelayOn()
{
  digitalWrite(RELAY_PIN, HIGH);
  relayStatus = true;
}

void RelayOff()
{
  digitalWrite(RELAY_PIN, LOW);
  relayStatus = false;
}

void lcdPrint()
{
  if ((msNow < nextLcdTime))
    return;
  nextLcdTime += LCD_TIMEOUT;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(array1);
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.setCursor(2, 1);
  lcd.print(Input, 0);
  lcd.setCursor(6, 1);
  lcd.print("R:");
  lcd.setCursor(8, 1);
  lcd.print(Setpoint, 0);
  lcd.setCursor(12, 1);
  switch (plateMode)
  {
  case MODE0:
    lcd.print("M0");
    break;
  case MODE1:
    lcd.print("M1");
    break;
  default:
    lcd.print("OFF");
    break;
  }
  if (relayStatus)
  {
    lcd.setCursor(15, 1);
    lcd.print('*');
  }
}

void readTemperature()
{
  Input = thermistor->readCelsius();
}

void checkKeys()
{
  if ((msNow < nextKeyTime))
    return;
  nextKeyTime += KEY_TIMEOUT;
  if (digitalRead(KEY_PLUS_PIN) == LOW)
  {
    Setpoint++;
    toneKeyPress();
  }
  if (digitalRead(KEY_MINUS_PIN) == LOW)
  {
    Setpoint--;
    toneKeyPress();
  }
  if (digitalRead(KEY_OK_PIN) == LOW)
  {
    plateMode = MODE0;
    toneKeyPress();
  }
  if (digitalRead(KEY_BACK_PIN) == LOW)
  {
    plateMode = OFF;
    toneKeyPress();
  }
}

//*****************    PID    ************
#define PID_TIMEOUT_MS 100
#define PID_ELAPSED_TIME_S (float)PID_TIMEOUT_MS / 1000.0
#define RELAY_DEBOUNCE_MS 500
#define PID_MIN 0
#define PID_MAX 350
#define PID_OUTPUT_RELAY_BOUND 100
float Kp = 0.1, Ki = 0.1, Kd = 0.01;
const unsigned long windowSize = 5000;
float dP, dI, dD;
float errorPID, pervErrorPID;

inline float limit(float *val)
{
  return (*val > PID_MAX) ? PID_MAX : (*val < PID_MIN) ? PID_MIN
                                                       : *val;
}

void checkPID()
{
  if (msNow < PID_TIMEOUT_MS)
    return;
  windowStartTime += PID_TIMEOUT_MS;
  pervErrorPID = errorPID;
  errorPID = Setpoint - Input;
  dP = Kp * errorPID;
  dI += Ki * errorPID * PID_ELAPSED_TIME_S;
  dD = Kd * (errorPID - pervErrorPID) / PID_ELAPSED_TIME_S;
  dI = limit(&dI);
  Output = dP + dI + dD;
  Output = limit(&Output);

  if (!relayStatus && Output > PID_OUTPUT_RELAY_BOUND)
  {
    if (msNow > nextSwitchTime)
    {
      nextSwitchTime = msNow + RELAY_DEBOUNCE_MS;
      RelayOn();
    }
  }
  else if (relayStatus && Output < PID_OUTPUT_RELAY_BOUND)
  {
    if (msNow > nextSwitchTime)
    {
      nextSwitchTime = msNow + RELAY_DEBOUNCE_MS;
      RelayOff();
    }
  }
}

//**************        DEBUG   ***************


#define DEBUG_TIMEOUT 500
unsigned long nextDebugPrintTime = 0;
void debugPrint()
{
  if (msNow < nextDebugPrintTime)
    return;
  nextDebugPrintTime += 500;
  Serial.print(Setpoint);
  Serial.print('\t');
  Serial.print(Input);
  Serial.print('\t');
  Serial.print(Output);
  Serial.print('\t');
  Serial.print(dP * 100);
  Serial.print('\t');
  Serial.print(dI * 100);
  Serial.print('\t');
  Serial.print(dD * 100);
  Serial.println();
}

//************      SETUP   ********************
void setup()
{
  Serial.begin(115200);
  lcd.begin(16, 2);
  pinMode(CONTRAST_PIN, OUTPUT);
  pinMode(LCD_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(KEY_OK_PIN, INPUT_PULLUP);
  pinMode(KEY_PLUS_PIN, INPUT_PULLUP);
  pinMode(KEY_MINUS_PIN, INPUT_PULLUP);
  pinMode(KEY_BACK_PIN, INPUT_PULLUP);

  analogWrite(CONTRAST_PIN, contrast);
  analogWrite(LCD_LED_PIN, ledBright);
  toneOn();
  delay(500);
  toneOff();
  thermistor = new SmoothThermistor(
      new NTC_Thermistor(
          SENSOR_PIN,
          REFERENCE_RESISTANCE,
          NOMINAL_RESISTANCE,
          NOMINAL_TEMPERATURE,
          B_VALUE),
      SMOOTHING_FACTOR);
}

void loop()
{
  msNow = millis();
  checkKeys();
  readTemperature();
  lcdPrint();
#ifdef DEBUG
  debugPrint();
#endif
  switch (plateMode)
  {
  case OFF:
    RelayOff();
    break;
  case MODE0: // Режим преднагрева. Выход на заданную температуру и удержание
    checkPID();
    break;
  case MODE1: // TODO: автоматический режим кривой оплавления
    break;

  default:
    break;
  }
}
