#ifndef SOLDER_H
#define SOLDER_H

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
    INIT,
    MODE0,
    MODE1,
    MODE2
};

enum SolderModeEnum
{
    Start = 0,
    Preheat,
    Soak,
    Reflow,
    ReflowHold,
    Cooling,
    Stop
};

PlateMode &operator++(PlateMode &stackID)
{
    switch (stackID)
    {
    case MODE0:
        return stackID = MODE1;
    case MODE1:
        return stackID = MODE2;
    default:
        return stackID = MODE0;
    }
}
PlateMode operator++(PlateMode &stackID, int)
{
    PlateMode tmp(stackID);
    ++stackID;
    return tmp;
}

SolderModeEnum &operator++(SolderModeEnum &stackID)
{
    switch (stackID)
    {
    case Start:
        return stackID = Preheat;
    case Preheat:
        return stackID = Soak;
    case Soak:
        return stackID = Reflow;
    case Reflow:
        return stackID = ReflowHold;
    case ReflowHold:
        return stackID = Cooling;
    case Cooling:
        return stackID = Stop;
    case Stop:
        return stackID = Start;
    }
    return stackID;
}
SolderModeEnum operator++(SolderModeEnum &stackID, int)
{
    SolderModeEnum tmp(stackID);
    ++stackID;
    return tmp;
}

#endif