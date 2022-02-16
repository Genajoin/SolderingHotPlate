#include <Arduino.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 2, 4, 6, 7);
int contrastPin = 3; // PD3
int lcdLedPin = 9;   // PB1
int buzzerPin = 5;   // PD5
int relayPin = 8;    // PB0
int termPin = A0;
int contrast = 116;
int ledBright = 100;
int term = 0;
char array1[] = "HeatPlateAlpisto";
int tim = 500;

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
  lcd.print(term);
  lcd.setCursor(5, 1);
  lcd.print(contrast);
  lcd.setCursor(9, 1);
  lcd.print(ledBright);
}

void readTemperature()
{
  term = analogRead(termPin);
}

void setup()
{
  Serial.begin(115200);
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
}

void loop()
{
  readTemperature();
  Serial.println(term);
  lcdPrint();
  // analogWrite(lcdLedPin, ledBright++);
  delay(tim);
}