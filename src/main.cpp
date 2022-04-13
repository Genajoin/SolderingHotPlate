#include <Arduino.h>
#include "LiquidCrystal.h" // https://github.com/arduino-libraries/LiquidCrystal
#include "main.h"

PlateMode plateMode = INIT;
#ifdef DEBUG
SolderModeEnum solderMode = Start;
float Setpoint = 100.0;
#else
SolderModeEnum solderMode = Stop;
float Setpoint = 150.0;
#endif
int contrast = 116;  // init pwm [0;255]
int ledBright = 100; // init pwm [0;255]
const char array1[] = "HeatPlateAlpisto";
const String selderModeMessage[7] = {"Start",
                                     "Preheat",
                                     "Soak",
                                     "Reflow",
                                     "ReflowHold",
                                     "Cooling",
                                     "Stop"};
const String plateModeMessage[4] = {"INIT",
                                    "M0",
                                    "M1",
                                    "M2"};
unsigned long msNow;
unsigned long windowStartTime, nextSwitchTime, nextLcdTime, nextKeyTime, modeStopTimeout, modeStartTimeout;
float Input, Output;
bool relayStatus = false;
LiquidCrystal lcd(12, 11, 2, 4, 6, 7);
void debugPrintNow();

//**************    UTIL   ***************

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
  lcd.print(selderModeMessage[solderMode]);
  lcd.setCursor(12, 0);
  lcd.print(msNow > modeStopTimeout ? 0 : (modeStopTimeout - msNow) / 1000);
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.setCursor(2, 1);
  lcd.print(Input, 0);
  lcd.setCursor(6, 1);
  lcd.print("R:");
  lcd.setCursor(8, 1);
  lcd.print(Setpoint, 0);
  lcd.setCursor(12, 1);
  lcd.print(plateModeMessage[plateMode]);
  if (relayStatus)
  {
    lcd.setCursor(15, 1);
    lcd.print('*');
  }
}

//********************         TERMISTOR    **************
// расчет через табличную апроксимацию
// источник https://aterlux.ru/article/ntcresistor

// Значение температуры, возвращаемое если сумма результатов АЦП больше первого значения таблицы
#define TEMPERATURE_UNDER 0
// Значение температуры, возвращаемое если сумма результатов АЦП меньше последнего значения таблицы
#define TEMPERATURE_OVER 3000
// Значение температуры соответствующее первому значению таблицы
#define TEMPERATURE_TABLE_START 50
// Шаг таблицы
#define TEMPERATURE_TABLE_STEP 50

// Тип каждого элемента в таблице, если сумма выходит в пределах 16 бит - uint16_t, иначе - uint32_t
typedef uint16_t temperature_table_entry_type;
// Тип индекса таблицы. Если в таблице больше 256 элементов, то uint16_t, иначе - uint8_t
typedef uint8_t temperature_table_index_type;
// Метод доступа к элементу таблицы, должна соответствовать temperature_table_entry_type
#define TEMPERATURE_TABLE_READ(i) pgm_read_word(&termo_table[i])

const temperature_table_entry_type termo_table[] PROGMEM = {
    64478, 64166, 63780, 63307, 62735, 62050, 61239, 60290, 59193, 57938, 56521, 54941, 53200, 51308, 49278, 47127, 44879, 42557, 40190, 37805, 35429, 33087, 30802, 28594, 26477, 24464, 22562, 20778, 19111, 17562, 16129, 14807, 13591, 12475, 11453, 10519, 9665, 8887, 8176, 7529, 6939, 6401, 5910, 5462, 5054, 4681, 4340, 4028, 3743, 3482, 3242, 3022, 2820, 2635, 2464, 2307, 2162, 2028, 1904, 1790};

// Функция вычисляет значение температуры в десятых долях градусов Цельсия
// в зависимости от суммарного значения АЦП.
int16_t calc_temperature(temperature_table_entry_type adcsum)
{
  temperature_table_index_type l = 0;
  temperature_table_index_type r = (sizeof(termo_table) / sizeof(termo_table[0])) - 1;
  temperature_table_entry_type thigh = TEMPERATURE_TABLE_READ(r);

  // Проверка выхода за пределы и граничных значений
  if (adcsum <= thigh)
  {
#ifdef TEMPERATURE_UNDER
    if (adcsum < thigh)
      return TEMPERATURE_UNDER;
#endif
    return TEMPERATURE_TABLE_STEP * r + TEMPERATURE_TABLE_START;
  }
  temperature_table_entry_type tlow = TEMPERATURE_TABLE_READ(0);
  if (adcsum >= tlow)
  {
#ifdef TEMPERATURE_OVER
    if (adcsum > tlow)
      return TEMPERATURE_OVER;
#endif
    return TEMPERATURE_TABLE_START;
  }

  // Двоичный поиск по таблице
  while ((r - l) > 1)
  {
    temperature_table_index_type m = (l + r) >> 1;
    temperature_table_entry_type mid = TEMPERATURE_TABLE_READ(m);
    if (adcsum > mid)
    {
      r = m;
    }
    else
    {
      l = m;
    }
  }
  temperature_table_entry_type vl = TEMPERATURE_TABLE_READ(l);
  if (adcsum >= vl)
  {
    return (l * TEMPERATURE_TABLE_STEP + TEMPERATURE_TABLE_START);
  }
  temperature_table_entry_type vr = TEMPERATURE_TABLE_READ(r);
  temperature_table_entry_type vd = vl - vr;
  int16_t res = (TEMPERATURE_TABLE_START + r * TEMPERATURE_TABLE_STEP);
  if (vd)
  {
    // Линейная интерполяция
    res -= ((TEMPERATURE_TABLE_STEP * (int32_t)(adcsum - vr) + (vd >> 1)) / vd);
  }
  return res;
}

#define SENSOR_PIN A0
#define ADC_AVERAGE_MAX 64
uint16_t adcSensorValue;
uint8_t adcCount = 0;

// основная версия получения температуры
void readTemperature()
{
  adcSensorValue += analogRead(SENSOR_PIN);
  if (++adcCount < ADC_AVERAGE_MAX)
    return;
  int16_t t = calc_temperature(adcSensorValue);
  adcCount = 0;
  adcSensorValue = 0;
  Input = (float)t / 10.f;
}

/// Версия прямого расчета температуры по формуле
// + просто
// - медленно за счет log & float, неточно на больших температурах
#define REFERENCE_RESISTANCE 4700.f
#define NOMINAL_RESISTANCE 100000.f
#define NOMINAL_TEMPERATURE 25.f + 273.15f
#define B_VALUE 3950.f
#define SMOOTHING_FACTOR 12.f

void readTemperature_calcLog()
{
  float kelvin = REFERENCE_RESISTANCE / (((1023.f / (float)adcSensorValue)) - 1);
  kelvin = kelvin / NOMINAL_RESISTANCE;                // (R/Ro)
  kelvin = log(kelvin);                                // ln(R/Ro)
  kelvin = kelvin + (B_VALUE / (NOMINAL_TEMPERATURE)); // 1/B * ln(R/Ro)
  kelvin = B_VALUE / kelvin;                           // Invert
  float celsius = kelvin - 273.15f;
  celsius = (celsius > 0) ? celsius : 0;
  Input = (Input * (SMOOTHING_FACTOR - 1.0f) + celsius) / SMOOTHING_FACTOR;
}

//****************       KEY    ***********************
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
    //    plateMode = MODE0;
    solderMode++;
    toneKeyPress();
  }
  if (digitalRead(KEY_BACK_PIN) == LOW)
  {
    //    plateMode = OFF;
    plateMode++;
    toneKeyPress();
  }
}

//*****************    PID    ************
#define PID_TIMEOUT_MS 100
#define PID_ELAPSED_TIME_S (float)PID_TIMEOUT_MS / 1000.0
#define RELAY_DEBOUNCE_MS 100
// #define PID_OUTPUT_RELAY_BOUND 100
#define PID_MIN 0
#define PID_MAX 100.f
#define KDT 8.f
#define KVA0 6.f
#define KVA1 0.023f // v(T) = kva0 + kva1*T
// #define KTA0 (KVA0 * KDT) // T1 = T+v(T)*dt
// #define KTA1 (1.f + (KVA1 * KDT))

// https://www.rentanadviser.com/pid-fuzzy-logic/pid-fuzzy-logic.aspx
// float Kp = 0.1, Ki = 0.1, Kd = 0.01;
float Kp = 0.9, Ki = 0.2, Kd = 0.01;
float dP, dI, dD;
float errorPID, pervErrorPID, dInput, dIntegral;

inline float limit(float *val)
{
  return (*val > PID_MAX) ? PID_MAX : (*val < PID_MIN) ? PID_MIN
                                                       : *val;
}

void checkPID()
{
  if (msNow < windowStartTime)
    return;
  windowStartTime += PID_TIMEOUT_MS;
  pervErrorPID = errorPID;
  if (relayStatus)
  {
    dIntegral += (KVA0 - (KVA1 * Input)) * PID_ELAPSED_TIME_S * KDT;
  }
  else
  {
    dIntegral -= (KVA0 - (KVA1 * Input)) * PID_ELAPSED_TIME_S * dIntegral / 10.f;
  }
  dIntegral = dIntegral > 0 ? dIntegral : 0;
  dInput = Input + dIntegral;
  errorPID = Setpoint - dInput;
  dP = Kp * errorPID;
  dI += Ki * errorPID * PID_ELAPSED_TIME_S;
  dD = Kd * (errorPID - pervErrorPID) / dI;
  dI = limit(&dI);
  Output = dP + dI + dD;
  Output = limit(&Output);
  /////////////////////////////////////////
  if (!relayStatus && (dInput < Setpoint) && (Output > 0.1f))
  {
    RelayOn();
#ifdef DEBUG
    debugPrintNow();
#endif
  }
  else if (relayStatus && (dInput > Setpoint))
  {
    RelayOff();
#ifdef DEBUG
    debugPrintNow();
#endif
  }
}

// ======================   MODE   =========================
#define HEAT_DEBOUNCE_MS 2000
//                 Start,Preheat,  Soak,Reflow,ReflowHold,Cooling
// uint32_t modeTimeToHeat0ms[6] = {0, 60000, 120000, 1000, 1, 10000};
// float mode0T[6] = {20.f, 50.f, 65.f, 85.f, 95.f, 20.f};
uint32_t mode0ms[6] = {0, 60000, 120000, 1000, 60000, 20000};
float mode0T[6] = {20.f, 150.f, 165.f, 225.f, 235.f, 20.f};
float mode0TFrom[6] = {mode0T[0], mode0T[0], mode0T[1], mode0T[2], mode0T[3], mode0T[4]};

uint32_t nextSetT = 0;
bool setTemperatureByTime(float T0, float T1, uint32_t t0, uint32_t t1)
{
  if (nextSetT > msNow || t1 < msNow)
    return false;
  nextSetT = msNow + HEAT_DEBOUNCE_MS;
  Setpoint = T0 + ((T1 - T0) * (msNow - t0) / (t1 - t0));
  return true;
}

void mode0()
{
  if (solderMode == Start)
  {
    checkPID();
    return;
  }
  solderMode = Stop;
  RelayOff();
}

/*
Зона	          Свинец (Sn63 Pb37)
Разогреть	      до 150 °C за 60 с
Замочить	      от 150 °C до 165 °C за 120 с
Перекомпоновать	Пиковая температура от 225 °C до 235 °C, удержание в течение 20 с.
Охлаждение	    -4 °C/с или естественное охлаждение
*/
void modeSn63Pb37()
{
  if (solderMode == Stop)
  {
    RelayOff();
    return;
  }

  if (msNow > modeStopTimeout)
  {
    solderMode++;
    modeStopTimeout = msNow + mode0ms[solderMode];
    modeStartTimeout = msNow;
  }
  setTemperatureByTime(mode0TFrom[solderMode], mode0T[solderMode], modeStartTimeout, modeStopTimeout);
  checkPID();
}
// SolderModeEnum modeSn63Pb37(SolderModeEnum sNum)
// {
//   solderMode = sNum;
//   return modeSn63Pb37();
// }

/*
Зона	          Без свинца (SAC305)
Разогреть       до 150 °C за 60 с
Замочить		    от 150 °C до 180 °C за 120 с
Перекомпоновать	Пиковая температура от 245 °C до 255 °C, удержание в течение 15 с.
Охлаждение		  -4 °C/с или естественное охлаждение
*/
uint32_t mode2ms[6] = {0, 60000, 120000, 1000, 60000, 20000};
float mode2T[6] = {20.f, 150.f, 180.f, 245.f, 255.f, 20.f};
float mode2TFrom[6] = {mode2T[0], mode2T[0], mode2T[1], mode2T[2], mode2T[3], mode2T[4]};

void modeSAC305()
{
  if (solderMode == Stop)
  {
    RelayOff();
    return;
  }

  if (msNow > modeStopTimeout)
  {
    solderMode++;
    modeStopTimeout = msNow + mode2ms[solderMode];
    modeStartTimeout = msNow;
  }
  setTemperatureByTime(mode2TFrom[solderMode], mode2T[solderMode], modeStartTimeout, modeStopTimeout);
  checkPID();
}
//**************        DEBUG   ***************

#define DEBUG_TIMEOUT 1000
unsigned long nextDebugPrintTime = 0;
void debugPrintNow()
{
  Serial.print(Input, 1);
  Serial.print('\t');
  Serial.print(Setpoint, 1);
  Serial.print('\t');
  Serial.print(Output, 1);
  Serial.print('\t');
  Serial.print(dP);
  Serial.print('\t');
  Serial.print(dI);
  Serial.print('\t');
  Serial.print(dD);
  Serial.print('\t');
  Serial.print(errorPID);
  Serial.print('\t');
  Serial.print(dInput);
  Serial.print('\t');
  Serial.print(dIntegral);
  Serial.printf("\t%d\t%d\r\n", relayStatus, solderMode);
}
void debugPrint()
{
  if (msNow < nextDebugPrintTime)
    return;
  nextDebugPrintTime += DEBUG_TIMEOUT;
  debugPrintNow();
}
//************      SETUP   ********************
void setup()
{
  Serial.begin(57600);
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
  lcd.print(array1);
  toneOn();
  delay(500);
  toneOff();
  delay(3000);
}

void loop()
{
  msNow = millis();
  checkKeys();
  readTemperature();
  lcdPrint();
  debugPrint();
  switch (plateMode)
  {
  case MODE0: // Режим преднагрева. Выход на заданную температуру и удержание
    mode0();
    break;
  case MODE1: // автоматический режим кривой оплавления
    modeSn63Pb37();
    break;
  case MODE2: // автоматический режим кривой оплавления
    modeSAC305();
    break;

  default:
    if (Input != 0.f)
    {
#ifdef DEBUG
      plateMode = MODE1;
      // Setpoint = Input + 10;
#else
      plateMode = MODE1;
#endif
    }
    break;
  }
}
