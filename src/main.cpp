/*
===============================================================================
  PressureController.ino

  Hydrogen Pressure Controller
  Target MCU : ATmega328P (Arduino Nano)
  Clock      : 16 MHz
  Serial     : 19200 baud, 8N1
  PWM        : Timer1 OC1A (D9), Fast PWM, 10 kHz
  Sensor     : I2C, address 0x20

  Pressure units:
      Internal : mbar (integer)
      Serial   : bar with 3 decimal digits

  Control algorithm:

      Error = Setpoint - Pressure

      if (Setpoint == 0)
          PWM = 0;

      else if (Error <= Deadband)
          PWM = 0;

      else
      {
          PWM = StartPWM + Error * K;

          PWM <= StartPWM + MaxPWMDifference
          PWM <= FullOpenPWM
      }

  EEPROM layout

      Address     Type        Variable

      0           uint32_t    Signature
      4           uint32_t    ValveStartPWM
      8           uint32_t    ValveFullPWM
      12          float       OpeningCoefficient
      16          uint32_t    MaximumPWMDifference
      20          uint32_t    MaximumPressure_mbar
      24          uint32_t    Deadband_mbar

===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>


//=============================================================================
// Hardware
//=============================================================================

static_assert(F_CPU == 16000000UL,
              "Firmware requires 16 MHz ATmega328P");
              
constexpr uint8_t PWM_PIN              = 9;      // OC1A (Timer1)
constexpr uint16_t TIMER1_TOP = 199;

constexpr uint8_t SENSOR_I2C_ADDRESS   = 0x20;


//=============================================================================
// Serial
//=============================================================================

constexpr uint32_t SERIAL_BAUD         = 19200;

constexpr uint8_t SERIAL_BUFFER_SIZE   = 32;

constexpr uint8_t SERIAL_PROCESS_LIMIT      = 16;

constexpr uint16_t SERIAL_TIMEOUT_MS = 150;


//=============================================================================
// Timing
//=============================================================================

constexpr uint16_t CONTROL_PERIOD_MS       = 100;
constexpr uint16_t SENSOR_CONVERSION_MS    = 10;


//=============================================================================
// EEPROM
//=============================================================================

constexpr uint32_t EEPROM_SIGNATURE = 0x48505247UL;      // "HPRG"

constexpr int EE_SIGNATURE              = 0;
constexpr int EE_START_PWM              = 4;
constexpr int EE_FULL_PWM               = 8;
constexpr int EE_OPENING_COEFFICIENT    = 12;
constexpr int EE_MAX_PWM_DIFFERENCE     = 16;
constexpr int EE_MAX_PRESSURE           = 20;
constexpr int EE_DEADBAND               = 24;


//=============================================================================
// Default configuration
//=============================================================================

constexpr uint8_t DEFAULT_START_PWM          = 0x20;
constexpr uint8_t DEFAULT_FULL_PWM           = 0x80;
constexpr float   DEFAULT_OPENING_K          = 1.0f;
constexpr uint8_t DEFAULT_MAX_PWM_DIFF       = 0x10;
constexpr uint16_t DEFAULT_MAX_PRESSURE      = 3000;
constexpr uint16_t DEFAULT_DEADBAND          = 5;


//=============================================================================
// Limits
//=============================================================================

constexpr uint8_t PWM_MIN = 0;
constexpr uint8_t PWM_MAX = 255;


//=============================================================================
// EEPROM configuration structure
//=============================================================================

struct ControllerConfiguration
{
    uint32_t Signature;

    uint32_t ValveStartPWM;
    uint32_t ValveFullPWM;

    float OpeningCoefficient;

    uint32_t MaximumPWMDifference;
    uint32_t MaximumPressure;

    uint32_t Deadband;
};


//=============================================================================
// Runtime controller state
//=============================================================================

struct ControllerState
{
    int16_t Pressure_mbar = 0;

    uint16_t Setpoint_mbar = 0;

    uint8_t CurrentPWM = 0;

    bool ConversionPending = false;

    unsigned long ConversionStartTime = 0;

    unsigned long LastConversionRequest = 0;

    bool ReplyPending = false;

    unsigned long LastSerialCharacterTime = 0;
};


static ControllerConfiguration Config;
static ControllerState State;


//=============================================================================
// Serial parser
//=============================================================================

char SerialBuffer[SERIAL_BUFFER_SIZE];

uint8_t SerialIndex = 0;


//=============================================================================
// Function prototypes
//=============================================================================

// EEPROM

void LoadConfiguration();
void SaveConfiguration();
void LoadDefaultConfiguration();
bool ValidateConfiguration();


// PWM

void InitializePWM();
void SetValvePWM(uint8_t pwm);


// Sensor

void RequestConversion();
int16_t ReadSensorValue();


// Controller

void RunPressureController();
void CalculatePWM();


// Serial

void ProcessSerial();
void ProcessCommand(const char *command);

bool ParsePressureValue(const char *text,
                        uint16_t &pressure_mbar);

void QueueReply();
void SendPendingReply();

void PrintPressureBar(uint16_t pressure_mbar);

void CheckSerialTimeout();


/*
// Utility

template<typename T>
T Clamp(T value, T minimum, T maximum);

*/

//=============================================================================
// Setup
//=============================================================================

void setup()
{
    //---------------------------------------------------------------------
    // Initialize runtime state
    //---------------------------------------------------------------------

    State.Pressure_mbar          = 0;
    State.Setpoint_mbar          = 0;
    State.CurrentPWM             = 0;

    State.ConversionPending      = false;

    State.ConversionStartTime    = 0;
    State.LastConversionRequest  = 0;

    State.ReplyPending           = false;

    SerialIndex                  = 0;


    //---------------------------------------------------------------------
    // Serial
    //---------------------------------------------------------------------

    Serial.begin(SERIAL_BAUD);

    // Make sure transmit buffer is empty before continuing.
    // Serial.flush();


    //---------------------------------------------------------------------
    // I2C
    //---------------------------------------------------------------------

    Wire.begin();

    // Default I2C speed (100 kHz)
    // Wire.setClock(100000);      // Not required


    //---------------------------------------------------------------------
    // Load controller configuration
    //---------------------------------------------------------------------

    LoadConfiguration();


    //---------------------------------------------------------------------
    // Initialize PWM hardware
    //---------------------------------------------------------------------

    InitializePWM();

    SetValvePWM(0);


    //---------------------------------------------------------------------
    // Start first pressure conversion immediately
    //---------------------------------------------------------------------

    RequestConversion();

    State.ConversionPending     = true;
    State.ConversionStartTime   = millis();
    State.LastConversionRequest = State.ConversionStartTime;
}

//=============================================================================
// Main loop
//=============================================================================

void loop()
{
    //---------------------------------------------------------------------
    // Highest priority
    //---------------------------------------------------------------------

    RunPressureController();


    //---------------------------------------------------------------------
    // Receive and decode serial commands
    //---------------------------------------------------------------------

    ProcessSerial();
    CheckSerialTimeout();


    //---------------------------------------------------------------------
    // Lowest priority
    //---------------------------------------------------------------------

    if (State.ReplyPending)
    {
        SendPendingReply();
    }
}

//=============================================================================
// EEPROM
//=============================================================================

void LoadConfiguration()
{
    EEPROM.get(EE_SIGNATURE, Config.Signature);

    // Check initialization signature
    if (Config.Signature != EEPROM_SIGNATURE)
    {
        LoadDefaultConfiguration();
        SaveConfiguration();
        return;
    }

    EEPROM.get(EE_START_PWM,           Config.ValveStartPWM);
    EEPROM.get(EE_FULL_PWM,            Config.ValveFullPWM);
    EEPROM.get(EE_OPENING_COEFFICIENT, Config.OpeningCoefficient);
    EEPROM.get(EE_MAX_PWM_DIFFERENCE,  Config.MaximumPWMDifference);
    EEPROM.get(EE_MAX_PRESSURE,        Config.MaximumPressure);
    EEPROM.get(EE_DEADBAND,            Config.Deadband);

    if (!ValidateConfiguration())
    {
        LoadDefaultConfiguration();
        SaveConfiguration();
    }
}


//-----------------------------------------------------------------------------

void SaveConfiguration()
{
    Config.Signature = EEPROM_SIGNATURE;

    EEPROM.put(EE_SIGNATURE,           Config.Signature);
    EEPROM.put(EE_START_PWM,           Config.ValveStartPWM);
    EEPROM.put(EE_FULL_PWM,            Config.ValveFullPWM);
    EEPROM.put(EE_OPENING_COEFFICIENT, Config.OpeningCoefficient);
    EEPROM.put(EE_MAX_PWM_DIFFERENCE,  Config.MaximumPWMDifference);
    EEPROM.put(EE_MAX_PRESSURE,        Config.MaximumPressure);
    EEPROM.put(EE_DEADBAND,            Config.Deadband);
}


//-----------------------------------------------------------------------------

void LoadDefaultConfiguration()
{
    Config.Signature = EEPROM_SIGNATURE;

    Config.ValveStartPWM        = DEFAULT_START_PWM;
    Config.ValveFullPWM         = DEFAULT_FULL_PWM;
    Config.OpeningCoefficient   = DEFAULT_OPENING_K;
    Config.MaximumPWMDifference = DEFAULT_MAX_PWM_DIFF;
    Config.MaximumPressure      = DEFAULT_MAX_PRESSURE;
    Config.Deadband             = DEFAULT_DEADBAND;
}


//-----------------------------------------------------------------------------

bool ValidateConfiguration()
{
    //---------------------------------------------------------------------
    // Valve start PWM
    //---------------------------------------------------------------------

    if (Config.ValveStartPWM > PWM_MAX)
    {
        return false;
    }

    //---------------------------------------------------------------------
    // Valve full-open PWM
    //---------------------------------------------------------------------

    if (Config.ValveFullPWM > PWM_MAX)
    {
        return false;
    }

    if (Config.ValveFullPWM < Config.ValveStartPWM)
    {
        return false;
    }

    //---------------------------------------------------------------------
    // Opening coefficient
    //---------------------------------------------------------------------

    if (!isfinite(Config.OpeningCoefficient))
    {
    return false;
    }

    if (Config.OpeningCoefficient < 0.0f)
    {
        return false;
    }

    if (Config.OpeningCoefficient > 20.0f)
    {
        return false;
    }

    //---------------------------------------------------------------------
    // Maximum PWM difference
    //---------------------------------------------------------------------

    if (Config.MaximumPWMDifference > PWM_MAX)
    {
        return false;
    }

    //---------------------------------------------------------------------
    // Maximum pressure
    //---------------------------------------------------------------------

    if (Config.MaximumPressure < 100)
    {
        return false;
    }

    if (Config.MaximumPressure > 5000)
    {
        return false;
    }

    //---------------------------------------------------------------------
    // Deadband
    //---------------------------------------------------------------------

    if (Config.Deadband > 100)
    {
        return false;
    }

    //---------------------------------------------------------------------
    // Additional consistency checks
    //---------------------------------------------------------------------

    uint16_t maximumPossiblePWM =
        Config.ValveStartPWM +
        Config.MaximumPWMDifference;

    if (maximumPossiblePWM > 255)
    {
        // This is not fatal, since the controller also limits
        // output by ValveFullPWM and PWM_MAX.
        // Clamp it to avoid accidental overflow later.

        Config.MaximumPWMDifference =
            255 - Config.ValveStartPWM;
    }

    return true;
}

//=============================================================================
// PWM
//=============================================================================

void InitializePWM()
{
    //---------------------------------------------------------------------
    // Configure output pin
    //---------------------------------------------------------------------

    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);

    //---------------------------------------------------------------------
    // Stop Timer1
    //---------------------------------------------------------------------

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    //---------------------------------------------------------------------
    // Fast PWM
    // Mode 14
    // TOP = ICR1
    //---------------------------------------------------------------------

    ICR1 = TIMER1_TOP;                 // 10 kHz

    OCR1A = 0;

    //---------------------------------------------------------------------
    // Non-inverting PWM on OC1A (D9)
    //---------------------------------------------------------------------

    TCCR1A |= (1 << COM1A1);

    //---------------------------------------------------------------------
    // Fast PWM mode 14
    //---------------------------------------------------------------------

    TCCR1A |= (1 << WGM11);

    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << WGM13);

    //---------------------------------------------------------------------
    // Prescaler = 8
    //---------------------------------------------------------------------

    TCCR1B |= (1 << CS11);
}


//-----------------------------------------------------------------------------

void SetValvePWM(uint8_t pwm)
{
    State.CurrentPWM = pwm;

    if (pwm == 0)
    {
        OCR1A = 0;
        return;
    }

    uint16_t compare = ((uint16_t)pwm * TIMER1_TOP + 127U) / 255U;

    if (compare > TIMER1_TOP)
    {
        compare = TIMER1_TOP;
    }


    OCR1A = compare;
}

//=============================================================================
// Controller
//=============================================================================

void RunPressureController()
{
    unsigned long now = millis();

    //---------------------------------------------------------------------
    // Waiting for conversion to complete
    //---------------------------------------------------------------------

    if (State.ConversionPending)
    {
        if ((unsigned long)(now - State.ConversionStartTime) >= SENSOR_CONVERSION_MS)
        {
            State.Pressure_mbar = ReadSensorValue();

            State.ConversionPending = false;

            CalculatePWM();
        }

        return;
    }

    //---------------------------------------------------------------------
    // Time for next measurement
    //---------------------------------------------------------------------

    if ((unsigned long)(now - State.LastConversionRequest) >= CONTROL_PERIOD_MS)
    {
        RequestConversion();

        State.ConversionPending = true;
        State.ConversionStartTime = now;
        State.LastConversionRequest = now;

        return;
    }
}


//-----------------------------------------------------------------------------

void CalculatePWM()
{
    uint16_t effectiveSetpoint;
    int16_t error;
    uint16_t pwmDelta;
    uint16_t requestedPWM;

    //---------------------------------------------------------------------
    // Zero setpoint
    //---------------------------------------------------------------------

    if (State.Setpoint_mbar == 0)
    {
        SetValvePWM(0);
        return;
    }

    //---------------------------------------------------------------------
    // Limit setpoint without modifying requested value
    //---------------------------------------------------------------------

    effectiveSetpoint = State.Setpoint_mbar;

    if (effectiveSetpoint > (uint16_t)Config.MaximumPressure)
    {
        effectiveSetpoint = (uint16_t)Config.MaximumPressure;
    }

    //---------------------------------------------------------------------
    // Calculate error
    //---------------------------------------------------------------------

    error = (int16_t)effectiveSetpoint - State.Pressure_mbar;

    //---------------------------------------------------------------------
    // Close valve if pressure reached
    //---------------------------------------------------------------------

    if (error <= (int16_t)Config.Deadband)
    {
        SetValvePWM(0);
        return;
    }

    //---------------------------------------------------------------------
    // Calculate proportional PWM increment
    //---------------------------------------------------------------------

    pwmDelta = (uint16_t)(((float)error * Config.OpeningCoefficient) + 0.5f);

    //---------------------------------------------------------------------
    // Limit maximum flow
    //---------------------------------------------------------------------

    if (pwmDelta > (uint16_t)Config.MaximumPWMDifference)
    {
        pwmDelta = (uint16_t)Config.MaximumPWMDifference;
    }

    //---------------------------------------------------------------------
    // Add valve opening offset
    //---------------------------------------------------------------------

    requestedPWM = (uint16_t)Config.ValveStartPWM + pwmDelta;

    //---------------------------------------------------------------------
    // Limit by configured full-open PWM
    //---------------------------------------------------------------------

    if (requestedPWM > (uint16_t)Config.ValveFullPWM)
    {
        requestedPWM = (uint16_t)Config.ValveFullPWM;
    }

    //---------------------------------------------------------------------
    // Hardware limit
    //---------------------------------------------------------------------

    if (requestedPWM > PWM_MAX)
    {
        requestedPWM = PWM_MAX;
    }

    //---------------------------------------------------------------------
    // Apply PWM
    //---------------------------------------------------------------------

    SetValvePWM((uint8_t)requestedPWM);
}

//=============================================================================
// Serial
//=============================================================================

void ProcessSerial()
{
    uint8_t processed = 0;

    while (Serial.available() &&
           (processed < SERIAL_PROCESS_LIMIT))
    {

        char c = (char)Serial.read();

State.LastSerialCharacterTime = millis();

processed++;

        //-------------------------------------------------------------
        // Idle state
        //-------------------------------------------------------------

        if (SerialIndex == 0)
        {
            if (c == 'A')
            {
                SerialBuffer[0] = 'A';
                SerialIndex = 1;
            }

            continue;
        }

        //-------------------------------------------------------------
        // New command starts before previous finished.
        //-------------------------------------------------------------

        if (c == 'A')
        {
            SerialBuffer[SerialIndex] = '\0';
            ProcessCommand(SerialBuffer);

            SerialBuffer[0] = 'A';
            SerialIndex = 1;

            continue;
        }

        //-------------------------------------------------------------
        // End of command
        //-------------------------------------------------------------

        if ((c == '\r') || (c == '\n'))
        {
            SerialBuffer[SerialIndex] = '\0';

            ProcessCommand(SerialBuffer);

            SerialIndex = 0;

            continue;
        }

        //-------------------------------------------------------------
        // Ignore non-printable characters
        //-------------------------------------------------------------

        if (((uint8_t)c < 32) || ((uint8_t)c > 126))
        {
            SerialIndex = 0;
            continue;
        }

        //-------------------------------------------------------------
        // Buffer full
        //-------------------------------------------------------------

        if (SerialIndex >= (SERIAL_BUFFER_SIZE - 1))
        {
            SerialBuffer[SerialIndex] = '\0';

            ProcessCommand(SerialBuffer);

            SerialIndex = 0;

            continue;
        }

        //-------------------------------------------------------------
        // Store character
        //-------------------------------------------------------------

        SerialBuffer[SerialIndex++] = c;
    }
}


//-----------------------------------------------------------------------------

void ProcessCommand(const char *command)
{
    //-------------------------------------------------------------
    // A
    //-------------------------------------------------------------

    if (strcmp(command, "A") == 0)
    {
        QueueReply();
        return;
    }

    //-------------------------------------------------------------
    // Must start with "A "
    //-------------------------------------------------------------

    if (strncmp(command, "A ", 2) != 0)
        return;

    uint16_t pressure;

    if (!ParsePressureValue(command + 2, pressure))
        return;

    State.Setpoint_mbar = pressure;

    QueueReply();
}


//-----------------------------------------------------------------------------

bool ParsePressureValue(const char *text,
                        uint16_t &pressure_mbar)
{
    pressure_mbar = 0;

    uint16_t integerPart = 0;
    uint16_t fractionalPart = 0;

    uint8_t fractionalDigits = 0;

    bool decimalSeen = false;

    if (*text == '\0')
        return false;

    while (*text)
    {
        char c = *text++;

        if (c == '.')
        {
            if (decimalSeen)
                return false;

            decimalSeen = true;
            continue;
        }

        if ((c < '0') || (c > '9'))
            return false;

        uint8_t digit = c - '0';

        if (!decimalSeen)
        {
            integerPart *= 10;
            integerPart += digit;

            if (integerPart > 3)
                return false;
        }
        else
        {
            if (fractionalDigits >= 3)
                return false;

            fractionalPart *= 10;
            fractionalPart += digit;

            fractionalDigits++;
        }
    }

    while (fractionalDigits < 3)
    {
        fractionalPart *= 10;
        fractionalDigits++;
    }

    pressure_mbar =
        integerPart * 1000 +
        fractionalPart;

    if (pressure_mbar > Config.MaximumPressure)
        pressure_mbar = Config.MaximumPressure;

    return true;
}


//-----------------------------------------------------------------------------

void QueueReply()
{
    State.ReplyPending = true;
}


//-----------------------------------------------------------------------------

void SendPendingReply()
{
    State.ReplyPending = false;

    Serial.print(F("A "));

    PrintPressureBar(State.Setpoint_mbar);

    Serial.print(' ');

    PrintPressureBar(State.Pressure_mbar);

    Serial.print("\r\n");
}


//-----------------------------------------------------------------------------

void PrintPressureBar(uint16_t pressure_mbar)
{
    uint16_t integerPart =
        pressure_mbar / 1000;

    uint16_t fractionalPart =
        pressure_mbar % 1000;

    Serial.print(integerPart);

    Serial.print('.');

    if (fractionalPart < 100)
        Serial.print('0');

    if (fractionalPart < 10)
        Serial.print('0');

    Serial.print(fractionalPart);
}

void CheckSerialTimeout()
{
    if (SerialIndex == 0)
        return;

    if (Serial.available())
        return;

    if ((millis() - State.LastSerialCharacterTime) < SERIAL_TIMEOUT_MS)
        return;

    SerialBuffer[SerialIndex] = '\0';

    ProcessCommand(SerialBuffer);

    SerialIndex = 0;
}

void RequestConversion()
{
  
}

int16_t ReadSensorValue()
{
  return 0;
}