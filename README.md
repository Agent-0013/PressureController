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
