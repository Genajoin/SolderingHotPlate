#include <Arduino.h>
#include "LiquidCrystal.h" // https://github.com/arduino-libraries/LiquidCrystal

#define CONTRAST_PIN 3
#define LCD_LED_PIN 9
#define BUZZER_PIN 5 // PD5
#define RELAY_PIN 8  // PB0
#define KEY_OK_PIN 10
#define KEY_PLUS_PIN 15
#define KEY_MINUS_PIN 16
#define KEY_BACK_PIN 17

#define KEY_TIMEOUT 200
#define LCD_TIMEOUT 1000

enum PlateMode
{
  OFF = 0,
  MODE0,
  MODE1
};

#ifdef DEBUG
PlateMode plateMode = MODE0;
float Setpoint = 285.0;
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

//********************         TERMISTOR    **************
// расчет через табличную апроксимацию
// источник https://aterlux.ru/article/ntcresistor

// Значение температуры, возвращаемое если сумма результатов АЦП больше первого значения таблицы
#define TEMPERATURE_UNDER 0
// Значение температуры, возвращаемое если сумма результатов АЦП меньше последнего значения таблицы
#define TEMPERATURE_OVER 300
// Значение температуры соответствующее первому значению таблицы
#define TEMPERATURE_TABLE_START 5
// Шаг таблицы
#define TEMPERATURE_TABLE_STEP 5

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
    return l * TEMPERATURE_TABLE_STEP + TEMPERATURE_TABLE_START;
  }
  temperature_table_entry_type vr = TEMPERATURE_TABLE_READ(r);
  temperature_table_entry_type vd = vl - vr;
  int16_t res = TEMPERATURE_TABLE_START + r * TEMPERATURE_TABLE_STEP;
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
  Input = t;
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
#define PID_OUTPUT_RELAY_BOUND 100
#define PID_MIN 0
#define PID_MAX PID_OUTPUT_RELAY_BOUND * 2
float Kp = 0.1, Ki = 0.1, Kd = 0.01;
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
  Serial.print(dP);
  Serial.print('\t');
  Serial.print(dI);
  Serial.print('\t');
  Serial.print(dD);
  Serial.println();
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
  toneOn();
  delay(500);
  toneOff();
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
