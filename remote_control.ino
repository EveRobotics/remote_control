/*****************************************************************************/
/**
 * Bauxie Remote Control v1.0
 *
 * Radio Message & Intended Motion
 *   The radio outputs an 12 char radio message with format "m:_,l:_,r:_;" 
 *   'm' indicates the mode where: 
 *     'm:0' = full stop (movement disabled); 
 *     'm:1' = autonomous movement enabled,and; 
 *     'm:2' = manual joystick control enabled.
 *   'l' and 'r' values are for manual joystick control and indicate velocities
 *    for the left and right motors and range from 0-255, where:
 *     (l:128,r:128) represents joystick in neutral position, all motors stopped;
 *     (l:255,r:255) represents joystick forward, both motors full speed ahead;
 *     (l:0,r:0)     represents joystick back, both motors in reverse full speed;
 *     (l:0,r:255)   represents joystick left, putting left motor in reverse and 
 *                   right motor forward (rotates CCW in place);
 *     (l:255,r:0)   represents joystick right, putting left motor forward and 
 *                   right motor in reverse (rotate CW in place).
 *    When joystick is forward+left, both motors are forward but right more than
 *    left so it is veering forward+left.
 *    When joystick is forward+right, both motors are forward but left more than
 *    right so it veers forward+right.
 *    When joystick is back+left, both motors are in reverse but right more than
 *    left so it is turning CW and backing up.
 *    When joystick is back+right, both motors are in reverse but left more than
 *    right so it is turning CCW and backing up.
 *    NOTE: translating skid steering (usually done with 2 sticks) into single 
 *          stick control presents an interesting situation: when approaching 
 *          hard left from a backward vector (ie, approaching 9 o'clock from 
 *          the 6 o'clock position), the bot will abruptly transition from going
 *          backwards with CW turning to spinning CCW (and vice-versa for the 
 *          other direction). In order to mitigate this abrupt change, we added
 *          a brief window of pivoting just prior to the transition to spinning 
 *          in place. So, when approaching the the 9 o'clock position on the 
 *          joystick, just prior to entering a spin-in-place motion, the left 
 *          motors will stop moving while the right motors continue to move, 
 *          creating a pivoting motion (and vice-versa for the other direction).
 *          Initial mapping (v1.0) of the joystick inputs to the output command
 *          values for the motors was based primarily on intuition and will need
 *          to undergo field testing/validation and likely will need adjustment.
 *
 * Remote Controller - Physical Layout & Operation
 *  The controller has 3 switches, 1 momentary push button, 1 joystick, 4 LEDs.
 *    Power (On/Off) Switch - located in center of controller.
 *    Dead-Man-Switch(DMS) Enable/Disable Switch- this switch is located on top-right 
 *      and has a large red cover to allow quickly putting it in the down-position.
 *      When this switch is down, DMS mode is enabled. The controller will transmit 
 *      "m:0"  UNLESS the push button is held down, in which case it transmits  "m:1".
 *      Holding down the button when transmitting "m:1" the green LED will be lit.  
 *      If the push button is not held down, "m0" is transmitted and the red LED is lit.
 *      When operating in manual-control or continuous-autonomous modes (see below)
 *      the red-covered DMS/E-Stop switch can serve as a quick emergency stop by
 *      simply closing the large red switch cover which will enable the DMS mode (note: 
 *      you would want to keep you finger off the push button in this situation
 *      as that would enable autonomous motion).
 *      When the DMS Enable/Disable switch is in the UP position, DMS mode is disabled.
 *      and Continuous-Autonomous mode is enabled.  The controller will transmit "m:1"
 *      and the blue LED is lit, UNLESS the system is in "HALT", in which case "m:0" 
 *      will be transmitted and the yellow LED will also be lit.  The push button will
 *      toggle the "HALT" state off/on (push it once, don't hold it down).
 *    Manual-Control Enable Switch- on upper left of controller with blue cover.
 *      This switch AND the DMS switch must BOTH be in the UP position to enable 
 *      manual-control mode.  When manual-control is enabled, the controller transmits
 *      "m:2" as well as the mapped "l:" and "r:" values from the joystick UNLESS
 *      the system is in "HALT", in which case "m:0" is transmitted and the yellow 
 *      LED is lit.  The push-button will toggle "HALT" off/on (just push once).
 *      When in manual mode the DMS switch can serve as an emergency stop (just flip
 *      the DMS switch DOWN and keep your finger off the pushbutton).
 *    Push-Button- located on the right of the controller
 *      When in DMS mode (ie, both mode switches down), the push-button allows "m:1"
 *        to be sent when the button is depressed, and sends "m:0" if the button is
 *        not being held down.
 *      When in continuous-autonomous mode (DMS switch up, manual switch down), the
 *        push-button toggles the "HALT" state.  When not halted, the blue LED is lit
 *        and "m:1" is transmitted.  When halted, the yellow LED is also lit and "m:0"
 *        is sent.
 *      When in manual-control mode (both DMS and manual switches up), the push-button
 *        similarly toggles the "HALT" state off/on.  When not halted, the blue LED is 
 *        lit and "m:2" is sent.  when in "HALT" the yellow LED is lit and "m:0" is sent.
 */

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#define DEBUG_PRINT 1

const unsigned char NETWORKID = 100; // must be the same for all nodes (0-255)
const unsigned char NODEID    = 2;   // my node ID (0-255)
const unsigned char RECEIVER  = 255;
const unsigned char FREQUENCY = RF69_915MHZ;
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
#define SERIAL_BAUD   57600
#define RFM69_CS      8
#define RFM69_IRQ     2
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9
#define LED           13  // onboard blinky

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

// Constants for pin numbers:
const int PIN_DEAD_MAN_EN_SW = 7;        // pin for DMS enable switch
const int PIN_DEAD_MAN_ON_LED = 3;       // pin to signal DMS 'Dead' state
const int PIN_DEAD_MAN_OFF_LED = A2;     // pin to signal DMS 'Alive' state
const int PIN_MOMENTARY_BUTTON = 6;      // pin for momentary push button
const int PIN_DEAD_MAN_DISABLED_LED = 4; // pin to signal DMS is disabled
const int PIN_HALT_LED = 5;              // pin to signal 'Halt' state is engaged
const int PIN_MANUAL_CTRL_SW = A3;       // pin to enable joystick manual control
const int PIN_JOYSTICK_X = A0;           // pin for joystick left/right
const int PIN_JOYSTICK_Y = A1;           // pin for joystick forward/reverse

const bool HALTED = true;
const bool NOT_HALTED = false;

enum RadioModes {
    NO_MOTION = 0,
    AUTONOMOUS = 1,
    HAND_CONTROL = 2,
};

const unsigned char SPEED_STOP = 128;
const unsigned char RADIO_PACKET_LEN = 12;

void sendControlOutput(unsigned char m, unsigned char l, unsigned char r) {
    unsigned char radiopacket[RADIO_PACKET_LEN];
    radiopacket[0] = 'm';
    radiopacket[1] = ':';
    radiopacket[2] = m; // mode
    radiopacket[3] = ',';
    radiopacket[4] = 'l';
    radiopacket[5] = ':';
    radiopacket[6] = l; // left motor speed
    radiopacket[7] = ',';
    radiopacket[8] = 'r';
    radiopacket[9] = ':';
    radiopacket[10] = r; // right motor speed
    radiopacket[11] = ';';

#if DEBUG_PRINT
    Serial.print("Sending: ");
    for(int i = 0; i < RADIO_PACKET_LEN; i++) {
        Serial.print(radiopacket[i]);
        Serial.print(",");
    }
    Serial.print("\n");
#endif

    //target node Id, message as string or byte array, message length
    if (radio.sendWithRetry(RECEIVER, radiopacket, RADIO_PACKET_LEN)) {

#if DEBUG_PRINT
        Serial.println("OK\n");
#endif

    } else {
        Serial.print("Error sending\n");
    }

    radio.receiveDone(); //put radio in RX mode
    Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
}

const int ZERO_THRESHOLD = 5;

void translateJoystickPositionToSpeed(int mappedX, int mappedY
        , unsigned char* motorLeft, unsigned char* motorRight) {

    *motorLeft = SPEED_STOP;
    *motorRight = SPEED_STOP;

    // Scale to +- 64
    mappedX = (mappedX - SPEED_STOP) / 2;
    mappedY = (mappedY - SPEED_STOP) / 2;

    if(abs(mappedX) < ZERO_THRESHOLD) {
        mappedX = 0;
    }

    if(abs(mappedY) < ZERO_THRESHOLD) {
        mappedY = 0;
    }

    if(mappedY > 0) {
        // Convert X, Y to left right for forward motion.
        *motorLeft = *motorLeft + mappedY + mappedX;
        *motorRight = *motorRight + mappedY - mappedX;
    } else if(mappedY < 0) {
        // For reverse motion.
        *motorLeft = *motorLeft + mappedY - mappedX;
        *motorRight = *motorRight + mappedY + mappedX;
    } else {
        // Zero turning radius turn:
        *motorLeft = *motorLeft + mappedX;
        *motorRight = *motorRight - mappedX;
    }

#if DEBUG_PRINT
    Serial.print("Joystick: X, Y: ");
    Serial.print(mappedX);
    Serial.print(", ");
    Serial.print(mappedY);
    Serial.print(", motor left, right: ");
    Serial.print(*motorLeft);
    Serial.print(", ");
    Serial.print(*motorRight);
    Serial.print("\n");
#endif

}


void setup() {
    // Initialize GPIO pins for their functions:
    pinMode(PIN_DEAD_MAN_EN_SW, INPUT);
    pinMode(PIN_DEAD_MAN_ON_LED, OUTPUT);
    pinMode(PIN_DEAD_MAN_OFF_LED, OUTPUT);
    pinMode(PIN_MOMENTARY_BUTTON, INPUT);
    pinMode(PIN_DEAD_MAN_DISABLED_LED, OUTPUT);
    pinMode(PIN_HALT_LED, OUTPUT);
    pinMode(PIN_MANUAL_CTRL_SW, INPUT);
    pinMode(PIN_JOYSTICK_X, INPUT);
    pinMode(PIN_JOYSTICK_Y, INPUT);

    // Initialize serial:
    Serial.begin(SERIAL_BAUD);
    Serial.println("Arduino RFM69HCW Transmitter");
    // Hard Reset the RFM module
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, HIGH);
    delay(100);
    digitalWrite(RFM69_RST, LOW);
    delay(100);
  
    // Initialize radio
    radio.initialize(FREQUENCY, NODEID, NETWORKID);
    if (IS_RFM69HCW) {
        radio.setHighPower();    // Only for RFM69HCW & HW!
    }
    radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
    radio.encrypt(ENCRYPTKEY);
    //pinMode(LED, OUTPUT);
    Serial.println("\nTransmitting at 915 MHz");
}

bool haltState = HALTED; // Variable for current halt state.

// Arduino main loop.
void loop() {

    unsigned char mode = NO_MOTION;
    unsigned char motorLeft = SPEED_STOP;
    unsigned char motorRight = SPEED_STOP;

    // Check if the DMS enable switch is on (red covered switch is in closed/down position):
    // Variable for whether the DMS is enabled.
    int dmsEnableState = digitalRead(PIN_DEAD_MAN_EN_SW);

    // Now check if DMS button is depressed:
    // Variable for reading momentary button
    int momentaryButtonPushed = digitalRead(PIN_MOMENTARY_BUTTON);

    // Check if manual control switch is on (blue covered switch is up):
    // Variable for joystick control.
    int joystickControlState = digitalRead(PIN_MANUAL_CTRL_SW);

    // No motion unless dead man button is held down continuously.
    if (dmsEnableState == LOW) {
        //Serial.print("DMS is enabled.");
        digitalWrite(PIN_DEAD_MAN_DISABLED_LED, LOW);  // turn off blue LED
        digitalWrite(PIN_HALT_LED, LOW); //  turn off yellow LED

        // If DMS is enabled and DMS button is depressed, cancel 'Die' signal 
        // and send 'Alive' signal:
        if (momentaryButtonPushed == HIGH) {
            digitalWrite(PIN_DEAD_MAN_ON_LED, LOW);   // turn off red LED
            digitalWrite(PIN_DEAD_MAN_OFF_LED, HIGH); // turn on green LED
            //Serial.print("\t DMS button is depressed (stay alive)\n");
            mode = AUTONOMOUS; // m1 allows autonomous motion
            // The system should disregard this value in m1, we but we send 128 anyway
            motorLeft = SPEED_STOP;
            // The system should disregard this value in m1, but we send 128 anyway
            motorRight = SPEED_STOP;
        } else {
            // If DMS is enabled and DMS is not depressed, cancel 'Alive' 
            // signal and send 'Die' signal:
            digitalWrite(PIN_DEAD_MAN_OFF_LED, LOW);  // turn off green LED
            digitalWrite(PIN_DEAD_MAN_ON_LED, HIGH);  // turn on red LED
            //Serial.print("\t DMS button is open (Die)\n");
            mode = NO_MOTION; // m0 disables all motion
            motorLeft = SPEED_STOP;
            motorRight = SPEED_STOP;
        }
    } else {
        // Motion is enabled and disabled by toggling the halt button (momentary press)
        // If the DMS is not enabled (red covered switch is uncovered and up),
        // turn off DMS LEDs, turn on 'DMS OFF' indicator:
        digitalWrite(PIN_DEAD_MAN_DISABLED_LED, HIGH);  // turn on blue LED
        digitalWrite(PIN_DEAD_MAN_OFF_LED, LOW); // Turn off green LED.
        digitalWrite(PIN_DEAD_MAN_ON_LED, LOW);  // Turn off red LED.
        //Serial.print("DMS is disabled.");
        // Check if halt button has been pushed, if so, toggle halt state:
        if (momentaryButtonPushed == HIGH) {
            haltState = !haltState;
            digitalWrite(PIN_HALT_LED, haltState);
            // For de-bouncing the push button.  don't hold it down for more
            // than half a second, push it once.
            delay(500);
        }

        if (joystickControlState == LOW && haltState == NOT_HALTED) {
            //Serial.print("\t Manual-Control Disabled. Continuous-Autonomous Enabled");
            // Autonomous movement enabled (m:1) if not halted, but (m:0) if it is halted.
            mode = AUTONOMOUS;
        } else if (joystickControlState == HIGH && haltState == NOT_HALTED) {
            // If DMS is off and manual control switch is on and not halted,
            // read and report joystick values:
            mode = HAND_CONTROL;
            int joystickX = analogRead(PIN_JOYSTICK_X);
            joystickX = map(joystickX, 0, 1023, 0, 255);
            int joystickY = analogRead(PIN_JOYSTICK_Y);
            joystickY = map(joystickY, 0, 1023, 0, 255);

            translateJoystickPositionToSpeed(joystickX, joystickY
                    , &motorLeft, &motorRight);
        }

    }

    // Send the motor speed and mode data out via the radio.
    sendControlOutput(mode, motorLeft, motorRight);
    delay(90); // This should match the loop timing for the spacial controller.
}


