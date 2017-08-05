/*
 *  DPV Controller
 *
 *  Controller software for a dive propulsion vehical.
 */

/* Pro Mini v3.0 Pinout
 *  
 *  IDE Pin  PCB Label
 *    0          RXD
 *    1          TXD
 *   2-13       D2-D13
 *  14-17       A0-A3
 *
 *  PWM IDE pin: 3, 5, 6, 9, 10, 11 (5,6 have higher than expected duty cycles)
 */


/*
 * Controller specific settings
 */

#define SWITCH_PIN 2    // D2
#define MOTOR_PIN 6     // D6 PWM 62500 Hz
#define VOLTAGE_PIN 14  // A0
#define CURRENT_PIN 15  // A1
#define BUZZER_PIN 3    // D3 PWM 31250 Hz
//#define TEMP_PIN 16   // A2
//#define UNUSED_PIN 17 // A3

/*
 * Battery Chemistry/Geometry settings
 */

#define BATT_WARN_VOLTAGE (12 * 3.0)
#define BATT_LIMP_VOLTAGE (12 * 2.3)  // Should be (12 * 2.4 but the voltage measurement varies too much and could trigger when it's 2.5)

/*
 * Speed settings, change speeds for different chemistry
 *
 * Voltage targets are 18, 24, and 30V, based on the old controller's performance
 * The original 12S NiMH batteries had a nominal voltage of 1.2V x 23 (27.6V)
 * and a max voltage of 34.5.
 * The 12S LiFePO4 batteries have a nominal voltage of 3.3V x 12 (39.6V)
 * and a max voltage of 43.8
 *
 * High speed: 30V/39.6 = 0.758
 * Med  speed: 24V/39.6 = 0.606
 * Low  speed: 18V/39.6 = 0.455
 */

#define SPEEDS			{ 0.45, 0.61, 0.76 }
//#define SPEEDS                  { 0.22, 0.30, 0.38 } // Bench test with motor gives these voltages, 
#define SPEED_PWM_FREQ_DIV       8  // About 1024=61Hz, 256=250Hz, 64=1KHz, 8=4KHz
#define SPEED_COUNT 3
#define MIN_SPEED 0.10			// Minimum speed (duty cycle) at which motor turns
#define START_SPEED_INDEX 1		// Index to starting speed
#define LIMP_SPEED_INDEX 0		// Index to limping speed

float currentSpeed = 0;			// Current speed (duty cycle)
float targetSpeed = 0;			// Target speed (duty cycle)
float speedStep = 0;			// Incremental increase/decrease in speed

/*
 * Common defines and variables
 */

#define LOOP_DELAY 100 // millis

enum { STATE_OFF, STATE_TO_ON, STATE_ON, STATE_TO_OFF };
float millis_factor = 1.0;


/*
 * Switch state
 */

unsigned int switchState = STATE_OFF;

/*
 * Motor state
 */

int motorState = STATE_OFF;
unsigned long motorOnTime = 0;  // when the motor transitions from STATE_TO_ON to STATE_ON.


/*
 * Voltage state
 */

#define VOLT_WARN_TIME	1000	// Duration of warning buzzer in ms
#define VOLT_WARN_DELAY	60000	// Delay between warnings

typedef enum {OKAY, WARN, LIMP} voltageState_type;
voltageState_type voltageState = OKAY;
unsigned long voltWarnTime = 0;

/*
 * Amperage state
 */

/*
 * Speed indicator state
 */

#define SPEED_IND_TIME	250		// Duration of indicator buzzer in ms
#define SPEED_IND_DELAY	2000	// Delay after motor on to start indication

unsigned int speedIndicator = 0;	// flag to start speed indication
unsigned int speedIndCount = 0;		// Number of buzzes to indicate
unsigned long speedIndTime = 0;		// Start of buzzer transition

/*
 * Buzzer state
 */

#define BUZZER_PWM_DUTY_CYCLE 1    // 24V buzzer
#define BUZZER_PWM_FREQ_DIV  1
int buzzerState = STATE_OFF;



// Return milliseconds taking into account timer0 changes
unsigned long true_millis() {
  return millis() * millis_factor;
}

// Perform delay taking into account timer0 changes.
void true_delay(int milliseconds) {
  delay(milliseconds / millis_factor);
}

/*
 * Each pin has a different base frequency
 *
 * Pins 3, 9, 10, and 11 are 31250 Hz (31KHz, 3.9KHz, 976.5Hz, 488Hz, 244Hz, 122Hz, 61Hz, 30.5Hz)
 * Pinx 5 and 6 are 62500 Hz (62.5Hz, 7.8KHz, 976.5Hz, 244Hz, 61Hz)
 *
 * We use a divisor to set frequency, each pin has different allow divisors
 *
 * Pins 3 and 11 allow 1 (001), 8 (010), 32 (011), 64 (100), 128 (101), 256 (110), and 1024 (111)
 * Pins 5, 6, 9, and 10 allow 1 (001), 8 (010), 64 (011), 256 (100), and 1024 (101)
 *
 * Frequencies are driven by timers which each control a pair of ports. So 
 * PWM frequency on paired ports will be identical
 *
 * Pins 3 and 11 use timer2 (TCCR2B)
 * Pins 5 and 6 use timer0 (TCCR0B)
 * Pins 9 and 10 use timer1 (TCCR1B)
 *
 * NOTE: 
 * Changes on pins 3, 5, 6, or 11 may affect the millis() and delay() function
 * Changes on pins 9 and 10 will cause the Servo library to function incorrectly.
 
 * 980 @ 60Hz
 * 510 @ 80Hz
 
 */

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
      // We have to adjust how millis() and delay() work
      millis_factor = divisor / 64.0;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

/*
*  setup()
*
*  Print program information,
*  Configure pins,
*  Set PWM frequency
*/

void setup() {
  Serial.begin(9600);
  Serial.println("");
  Serial.println("DPV Controller Software");
  Serial.println("Copyright (C) John Van Ostrand <john@vanostrand.com>.");
  Serial.println("");
  Serial.println("Designed for a Hollis H-160 DPV upgraded with A123 Systems");
  Serial.println("20Ah LiFePO4 prismatic batteries (12 in series).");
  Serial.println("");
  
  analogReference(DEFAULT);
 
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  
  setPwmFrequency(MOTOR_PIN, SPEED_PWM_FREQ_DIV);
  analogWrite(MOTOR_PIN, 0);
  
  pinMode(BUZZER_PIN, OUTPUT);
  //setPwmFrequency(BUZZER_PIN, BUZZER_PWM_FREQ_DIV);
  analogWrite(BUZZER_PIN, 0);

}


/*
*
*  setSpeed()
*
*  Convert the speed (a percentage) to analog output and write.
*
*/

void setSpeed(float speed) {
  // Determine PWM
  Serial.print("setSpeed(");
  Serial.print(speed);
  Serial.println(")");
  
  analogWrite(MOTOR_PIN, speed * 255);
}


/*
*
*  buzzer()
*
*  Turn buzzer on/off.
*
*/

void buzzer(int value) {
  if (value == HIGH) {
    Serial.println("Buzzer: On");
    buzzerState = STATE_ON;
    voltWarnTime = true_millis();
  } else {
    buzzerState = STATE_OFF;
    Serial.println("Buzzer: Off");
  }

  analogWrite(BUZZER_PIN, BUZZER_PWM_DUTY_CYCLE * 255 * value);
}


/*
*
*  resetBuzzer()
*
*  Turn buzzer off because of interruption.
*/

void resetBuzzer() {
  if (buzzerState == STATE_ON) 
    buzzer(LOW); // off
    
  voltWarnTime = 0;
  speedIndTime = 0;
}
    
  
/*
*
*  switchStateLoop()
*
*  Check switch. The switch acts like a signal to change speed.
*  When released (off) we wait 1 second before turning motor off.
*  If the switch is activated (on) before the one second is over
*  it signals the user wants a speed change. Speed is cycled from
*  2nd speed (22.5V) to 3rd speed (30V) to 1st speed (15V) and back to 
*  2nd speed. This is based on the author's preferences.
*
*/

void switchStateLoop() {
  float speeds[SPEED_COUNT] = SPEEDS;
  static unsigned int speedIndex = START_SPEED_INDEX;
  static unsigned long switchSignalTime = 0;

  // Check switch state
  int sw = digitalRead(SWITCH_PIN);

  switch (switchState) {
  case STATE_OFF:
    if (sw == LOW) {

      // Turn on
      Serial.println("Switch: On");
      
      switchState = STATE_ON;
      motorState = STATE_TO_ON;
      
      if (voltageState == LIMP)
        speedIndex = LIMP_SPEED_INDEX;
      else 
        speedIndex = START_SPEED_INDEX;
        
      currentSpeed = 0;
      targetSpeed = speeds[speedIndex];
      speedStep = (targetSpeed - currentSpeed) / (1000.0 / LOOP_DELAY);
      
      // Tell buzzer to indicate speed
      speedIndCount = speedIndex + 1;

      Serial.print("Speed change from ");
      Serial.print(currentSpeed);
      Serial.print(" to ");
      Serial.print(targetSpeed);
      Serial.print(" in ");
      Serial.print(speedStep);
      Serial.println(" incremenets.");
    }
    break;
  case STATE_ON:
    if (sw == HIGH) {
      // Switch off
      Serial.println("Switch: Off");
      
      resetBuzzer();
      
      if (voltageState == LIMP) {
        // Limp mode, just stop, signalling doesn't make sense
        switchState = STATE_OFF;
        currentSpeed = 0;
        targetSpeed = 0;
        setSpeed(targetSpeed);
      } else {
        // Go into signalling mode
        switchState = STATE_TO_OFF;
        switchSignalTime = true_millis();
      }
    }
    break;
  case STATE_TO_OFF:
    if (true_millis() - switchSignalTime >= 1000) {
      // Signal period timeout, turn off
      Serial.println("Switch: Off timeout");

      switchState = STATE_OFF;
      motorState = STATE_TO_OFF;
      resetBuzzer(); // No buzzer while off
    } else if (sw == LOW) {
      // Switch on, bump speed
      Serial.println("Switch: On, change speed");
      
      resetBuzzer(); // No buzzer while motor is STATE_TO_ON
      
      switchState = STATE_ON;
      motorState = STATE_TO_ON;
      speedIndex++;
      if (speedIndex >= SPEED_COUNT) speedIndex = 0;

      targetSpeed = speeds[speedIndex];
      
      // Spin up the motor in 1 second.
      speedStep = (targetSpeed - currentSpeed) /  (1000.0 / LOOP_DELAY);

      // Tell buzzer to indicate speed
      speedIndCount = speedIndex + 1;
      
      Serial.print("Speed change from ");
      Serial.print(currentSpeed);
      Serial.print(" to ");
      Serial.print(targetSpeed);
      Serial.print(" in ");
      Serial.print(speedStep);
      Serial.println(" incremenets.");
    }
    break;
  }  
}


/*
*
*  Control motor
*
*  A status of STATE_ON and STATE_OFF should be self explanatory.
*  A status of STATE_TO_OFF allows for delayed spin-down of the motor, the delay allows signalling of speed change.
*  A status of STATE_TO_ON allows for gradual spin-up for the motor.
*
*/

void motorControlLoop() {
  switch (motorState) {
  case STATE_TO_ON:
    // Gradually turn motor on
    if (currentSpeed == 0)
      currentSpeed = MIN_SPEED;
    else
      currentSpeed += speedStep;

    if ((speedStep > 0 && currentSpeed > targetSpeed) ||
        (speedStep < 0 && currentSpeed < targetSpeed)) {
      motorState = STATE_ON;
      currentSpeed = targetSpeed;
      // Flag to indicate speed.
      Serial.println("Setting speedIndicator");
      speedIndicator = 1;
    }
    setSpeed(currentSpeed);
    break;
  case STATE_TO_OFF:
    // Turn off motor immediately
    motorState = STATE_OFF;
    currentSpeed = 0;
    setSpeed(currentSpeed);
    break;
  }
}  


/*
*
*  monitorVoltageLoop()
*
*  Measure voltage and set voltage state.
*
*  The A123 LiFePo4 cells have the following properties
*
*  Fully Charged: 3.6V
*  Fully Discharged: 2.0v (0.5v before damage)
*  Nominal voltage: 3.3v
*
*  The voltages were chosen to warn at about 10% of burn time and go 
*  into limp mode with only a few minutes before 2.0V is reached.
*
*/

void monitorVoltageLoop() {
  
  const unsigned int sampleMax = 5;
  static float voltage[5];
  static int fullSample = 0;
  static int sampleCount = 0;
  float avgVoltage = 0;
  static float lastAvgVoltage = -1;


  // Check voltage
  voltage[sampleCount] = analogRead(VOLTAGE_PIN) * (5.0 / 1023.0) * (38.9 + 5.54) / 5.54; // Measured values
  voltage[sampleCount] *= 43.4/40.1;  // Calibration adjustment
  sampleCount++;

  if (sampleCount == sampleMax) {
     fullSample = 1;
     sampleCount = 0;
  }
  
  // Do nothing until we have 5 samples.
  if (! fullSample) return;
  
  float lowV = 9999, highV = 0;
  avgVoltage = 0;
  for (int x = 0; x < 5; x++) {
    if (voltage[x] > highV) highV = voltage[x];
    if (voltage[x] < lowV) lowV = voltage[x];
    avgVoltage += voltage[x];
  }
  avgVoltage /= 5;
  
  // Round
  avgVoltage = round(avgVoltage * 10.0) / 10.0;
  
  if (avgVoltage != lastAvgVoltage) {
    Serial.print("    avgVoltage: ");
    Serial.print(avgVoltage);
    Serial.print(" [ ");
    for (int x = 0; x < 5; x++) { Serial.print(voltage[x]); Serial.print(", "); }
    Serial.println("]");
    lastAvgVoltage = avgVoltage;
  }
    
  if (avgVoltage < BATT_LIMP_VOLTAGE && voltageState != LIMP) {
    // Turn off
    Serial.println("VoltageState: LIMP");
    voltageState = LIMP;
    motorState = STATE_OFF;
    switchState = STATE_OFF;
    currentSpeed = 0;
    setSpeed(0);
    
  } else if (avgVoltage < BATT_WARN_VOLTAGE && voltageState == OKAY) {
    // Start warning
    Serial.println("VoltageState: WARN");
    voltageState = WARN;
    voltWarnTime = true_millis() - VOLT_WARN_DELAY;
  } 
}


/*
*
*  monitorAmperageLoop()
*
*  Read amperage.
*
*/

void monitorAmperageLoop() {
  float amperage;
  float pinV, volts;
  static unsigned long now = 0, lastReport = 0;
  
  pinV = analogRead(CURRENT_PIN) / 1024 * 5;
  volts = pinV * (39 + 5.6) / 5.6;
  amperage = volts; /// 0.005;
  
  now = true_millis();
  if (now - lastReport > 10000) {
    lastReport = now;
    Serial.print("    Amperage: ");
    Serial.println(amperage);
  }
  if (amperage > 15) {
    // Cut-out
  }
}


/*
*
*  operateBuzzerLoop()
*
*  Operate the buzzer, called on low battery.
*
*/

void operateBuzzerLoop() {
  static unsigned long speedBuzzerTime = 0;
  unsigned long int now = true_millis();

  // Only warn when the scooter is on and not during a speed change.
  if (motorState != STATE_ON)
    return;
    
  // Do we need to indicate speed?
  // Wait SPEED_IND_DELAY before indication then alternate
  // SPEED_IND_TIME on and off for speedIndCount buzzes
  if (speedIndCount) {
    if (motorOnTime + SPEED_IND_DELAY < now) {
      if (speedIndicator) {
        // Initiate speed indication
        speedIndicator = 0;
        speedBuzzerTime = now;
      } else if (speedBuzzerTime + SPEED_IND_TIME <= now) {
        speedBuzzerTime = now;
        if (buzzerState == STATE_ON) {
          // Turn buzzer off
          buzzer(LOW);
          speedIndCount--;
          // Reset voltWarnTime to prevent a voltage warning for a little while
          if (speedIndCount == 0) {
            speedBuzzerTime = 0;
            voltWarnTime = now;
          }
        } else {
          // Turn buzzer on
          buzzer(HIGH);
        }
      }
    }
    return;
  }

  // Do we have a voltage warning?
  if (voltageState == WARN) {
    if (buzzerState == STATE_OFF) {  
      if (voltWarnTime + VOLT_WARN_DELAY <= now) {
        // Turn on buzzer
        buzzer(HIGH);
      }
    } else if (buzzerState == STATE_ON) {
      if (voltWarnTime + VOLT_WARN_TIME <= now) {
        // Turn off buzzer
        buzzer(LOW);
      }
    }
  }
} 
   

/*
*
*  loop()
*
*  Main loop
*
*/

void loop() {

  switchStateLoop();
  motorControlLoop();
  monitorVoltageLoop();
  monitorAmperageLoop();
  operateBuzzerLoop();
  
  true_delay(LOOP_DELAY);
}
