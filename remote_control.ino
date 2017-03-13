/**
 * Remote Controller
 
 * Crude prototype sketch for the Magellan bot remote control.
 * Has a selector switch which enables/disables the dead-man switch (DMS)
 * and a momentary button which must be held down during DMS mode for 
 * the bot to continue moving autonomously.
 */


// Constants for pin numbers:
const int dmsEnablePin = 7;       // pin for DMS enable switch
const int dmsDeadLED = 3;         // pin to signal DMS 'Dead' state
const int dmsAliveLED = 2;        // pin to signal DMS 'Alive' state
const int dmsLiveButton = 6;      // pin to receive 'StayAlive' button signal
const int dmsDisabledLED = 4;     // pin to signal DMS is disabled
const int haltLED = 5;            // pin to signal 'Halt' state is engaged
const int haltButton = 6;         // pin to receive 'Halt' button signal
const int manualControlPin = A3;  // pin to enable joystick manual control
const int joystickXPin = A0;      // pin for joystick left/right
const int joystickYPin = A1;      // pin for joystick forward/reverse


// Variables:
int dmsEnableState = 0;           // variable for whether the DMS is enabled
int dmsButtonState = 0;           // variable for whether the DMS button is depressed
int haltState = 1;                // variable for current halt state
int haltButtonState = 0;          // variable for reading halt button
int manualControlState = 0;       // variable for manual control
int joystickXValue = 0;           // variable for left/right
int joystickYValue = 0;           // variable for forward/reverse


void setup() {
    // Initialize GPIO pins for their functions:
    pinMode(dmsEnablePin, INPUT);
    pinMode(dmsDeadLED, OUTPUT);
    pinMode(dmsAliveLED, OUTPUT);
    pinMode(dmsLiveButton, INPUT);    // this will also function as the 'Halt' button
    pinMode(dmsDisabledLED, OUTPUT);
    pinMode(haltLED, OUTPUT);
    pinMode(manualControlPin, INPUT);
    pinMode(joystickXPin, INPUT);
    pinMode(joystickYPin, INPUT);

    // Initialize serial:
    Serial.begin(9600);
}

void loop() {
    // First check if the DMS enable switch is on:
    dmsEnableState = digitalRead(dmsEnablePin);
    
    if (dmsEnableState == LOW) {
        Serial.print("DMS is enabled.");
        digitalWrite(dmsDisabledLED, LOW);
        digitalWrite(haltLED, LOW);

        // Now check if DMS button is depressed:
        dmsButtonState = digitalRead(dmsLiveButton);

        // If DMS is enabled and DMS button is depressed, cancel 'Die' signal 
        // and send 'Alive' signal:
        if (dmsButtonState == HIGH) {
            digitalWrite(dmsDeadLED, LOW);
            digitalWrite(dmsAliveLED, HIGH);
            Serial.print("\t DMS button is closed (stay alive!)\n");
        } else {
            // If DMS is enabled and DMS is not depressed, cancel 'Alive' 
            // signal and send 'Die' signal:
            digitalWrite(dmsAliveLED, LOW);
            digitalWrite(dmsDeadLED, HIGH);
            Serial.print("\t DMS button is open (Die!)\n");
        }
    }
    if (dmsEnableState == HIGH) {
        
        // If the DMS is not enabled, turn off DMS LEDs, turn on 'DMS OFF' indicator:
        digitalWrite(dmsDisabledLED, HIGH);
        digitalWrite(dmsAliveLED, LOW);
        digitalWrite(dmsDeadLED, LOW);
        Serial.print("DMS is disabled.");
        
        // Check if halt button has been pushed, if so, toggle halt state:
        haltButtonState = digitalRead(haltButton);
        if (haltButtonState == HIGH) {
            haltState = !haltState;
            delay(500);
        }
        digitalWrite(haltLED, haltState);
        if (haltState == 1) {
            Serial.print("\t HALT!");
        }
        // Check if manual control switch is on:
        manualControlState = digitalRead(manualControlPin);
        if (manualControlState == LOW) {
            Serial.print("\t Manual Control Disabled.");
        } else {
            Serial.print("\t Manual Control Enabled.");
        }
        // If DMS is off and manual control switch is on and not halted, read 
        // and report joystick values:
        if (manualControlState == HIGH and haltState == LOW) {
            joystickXValue = analogRead(joystickXPin);
            joystickYValue = analogRead(joystickYPin);

            Serial.print("\t CW/CCW: ");
            Serial.print(joystickXValue);
            Serial.print(",\t Forward/Reverse: ");
            Serial.print(joystickYValue);
        }
        Serial.print("\n");
    }
}
