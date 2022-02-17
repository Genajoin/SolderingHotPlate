#include <Arduino.h>
#include "LiquidCrystal.h" // https://github.com/arduino-libraries/LiquidCrystal
#include "QuickPID.h"      // https://github.com/Dlloydev/QuickPID
#include <Thermistor.h>
#include <NTC_Thermistor.h> // https://github.com/YuriiSalimov/NTC_Thermistor

#define SENSOR_PIN A0
#define REFERENCE_RESISTANCE 4700
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950

const int contrastPin = 3; // PD3
const int lcdLedPin = 9;   // PB1
const int buzzerPin = 5;   // PD5
const int relayPin = 8;    // PB0
int contrast = 116;
int ledBright = 100;
const char array1[] = "HeatPlateAlpisto";
const int tim = 500;

// user settings
const unsigned long windowSize = 5000;
const byte debounce = 50;
float Input, Output, Setpoint = 200, Kp = 2, Ki = 5, Kd = 1;

// status
unsigned long windowStartTime, nextSwitchTime;
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
  analogWrite(buzzerPin, 128);
}
void toneOff()
{
  analogWrite(buzzerPin, 0);
}

void RelayOn()
{
  digitalWrite(relayPin, HIGH);
}

void RelayOff()
{
  digitalWrite(relayPin, LOW);
}

void lcdPrint()
{
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
  lcd.print(contrast);
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

void setup()
{
  // Serial.begin(115200);
  lcd.begin(16, 2);
  pinMode(contrastPin, OUTPUT);
  pinMode(lcdLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  analogWrite(contrastPin, contrast);
  analogWrite(lcdLedPin, ledBright);
  toneOn();
  RelayOn();
  delay(500);
  toneOff();
  RelayOff();
  thermistor = new NTC_Thermistor(
      SENSOR_PIN,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE);
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTimeUs(windowSize * 1000);
  myPID.SetMode(myPID.Control::automatic);
}

void loop()
{
  unsigned long msNow = millis();
  readTemperature();
  if (myPID.Compute())
    windowStartTime = msNow;

  if (!relayStatus && Output > (msNow - windowStartTime))
  {
    if (msNow > nextSwitchTime)
    {
      nextSwitchTime = msNow + debounce;
      relayStatus = true;
      digitalWrite(relayPin, HIGH);
    }
  }
  else if (relayStatus && Output < (msNow - windowStartTime))
  {
    if (msNow > nextSwitchTime)
    {
      nextSwitchTime = msNow + debounce;
      relayStatus = false;
      digitalWrite(relayPin, LOW);
    }
  }

  lcdPrint();
  delay(tim);
}
