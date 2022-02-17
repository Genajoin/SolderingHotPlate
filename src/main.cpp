#include <Arduino.h>
#include "LiquidCrystal.h" // https://github.com/arduino-libraries/LiquidCrystal
#include "QuickPID.h"      // https://github.com/Dlloydev/QuickPID
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

PlateMode plateMode = OFF;
int contrast = 116;
int ledBright = 100;
const char array1[] = "HeatPlateAlpisto";

unsigned long msNow;

// user settings
const unsigned long windowSize = 5000;
const byte debounce = 50;
float Input, Output, Setpoint = 200, Kp = 2, Ki = 5, Kd = 1;

// status
unsigned long windowStartTime, nextSwitchTime, nextLcdTime, nextKeyTime;
boolean relayStatus = false;
Thermistor *thermistor;
QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

LiquidCrystal lcd(12, 11, 2, 4, 6, 7);

void toneOn()
{
  analogWrite(BUZZER_PIN, 128);
}
void toneOff()
{
  analogWrite(BUZZER_PIN, 0);
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
  lcd.print("C:");
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
  }
  if (digitalRead(KEY_MINUS_PIN) == LOW)
  {
    Setpoint--;
  }
  if (digitalRead(KEY_OK_PIN) == LOW)
  {
    plateMode = MODE0;
  }
  if (digitalRead(KEY_BACK_PIN) == LOW)
  {
    plateMode = OFF;
  }
}

void checkPID()
{
  if (myPID.Compute())
    windowStartTime = msNow;

  if (!relayStatus && Output > (msNow - windowStartTime))
  {
    if (msNow > nextSwitchTime)
    {
      nextSwitchTime = msNow + debounce;
      RelayOn();
    }
  }
  else if (relayStatus && Output < (msNow - windowStartTime))
  {
    if (msNow > nextSwitchTime)
    {
      nextSwitchTime = msNow + debounce;
      RelayOff();
    }
  }
}

void setup()
{
  // Serial.begin(115200);
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
  // thermistor = new NTC_Thermistor(
  //     SENSOR_PIN,
  //     REFERENCE_RESISTANCE,
  //     NOMINAL_RESISTANCE,
  //     NOMINAL_TEMPERATURE,
  //     B_VALUE);
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTimeUs(windowSize * 1000);
  myPID.SetMode(myPID.Control::automatic);
}

void loop()
{
  msNow = millis();
  checkKeys();
  readTemperature();
  lcdPrint();

  switch (plateMode)
  {
  case OFF:
    RelayOff();
    break;
  case MODE0:
    checkPID();
    break;
  case MODE1:
    // TODO: автоматический режим кривой оплавления
    break;

  default:
    break;
  }
}
