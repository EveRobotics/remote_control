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
 *          creating a pivoting motion (and vice-versa for the other direciton).
 *          Initial mapping (v1.0) of the joystick inputs to the output command
 *          values for the motors was based primarily on intuition and will need
 *          to undergo field testing/validation and likely will need adjustment.
 *          The rationale for joystick mapping in v1.0 is described below.
 *
 *  Joystick Mapping 
 *    The space of possible joystick positions was divided into 19 sectors (0-18).  
 *    Because the joystick may not always return perfecly to center, the center
 *    point is expanded into a small central 'dead-zone' to signal no movement.
 *    We will call this central zone 'sector 0', then divide the remaining 
 *    joystick space into 18 radial zones (like pizza slices), numbering the
 *    slices starting from sector 1 at the 12:00 position and counting up in
 *    the CW direction. Sectors are defined by their boundry angles, and to 
 *    determine which sector the joystick is in we take the inverse tangent 
 *    of the ratio of the X and Y joystick values to give us the joystick angle.
 *    Note:The X and Y pots are mapped from (0,1023) to (0,255).
 *    For this chart, 12:00=0_degrees, 3:00=90, 6:00=180, and 9:00=270.
 *
 *    Sector     JoystickAngles  JoystickRange             Output(L,R)
 *    0          na               X:125-129, Y:125-129      L:128,R:128
 *    1           0-22
 *    2          23-45
 *    3          46-67
 *    4          68-79
 *    5          80-90
 *    6
 *    7
 *    8
 *    9
 *    10
 *    11
 *    12
 *    13
 *    14
 *    15
 *    16
 *    17
 *    18
 *
 * Remote Controller - Physical Layout & Operation
 *  The controller has 3 switches, 1 momentary push button, 1 joystick, 4 LEDs.
 *    Power (On/Off) Switch - located in center of controller.
 *    Dead-Man-Switch(DMS) Enable/Disable Switch- this switch is located on top-right 
 *      and has a large red cover to allow quicky putting it in the down-position.
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

//#define NETWORKID     100  // The same on all nodes that talk to each other
const unsigned char NETWORKID = 100; // must be the same for all nodes (0-255)
//#define NODEID        2    // The unique identifier of this node
const unsigned char NODEID = 2; // my node ID (0-255)
//#define RECEIVER      255    // The recipient of packets
const unsigned char RECEIVER = 255;
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
const unsigned char FREQUENCY = RF69_915MHZ;
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!

#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

#define SERIAL_BAUD   115200

#define RFM69_CS      8
#define RFM69_IRQ     2
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9
#define LED           13  // onboard blinky

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

// Constants for pin numbers:
const int dmsEnablePin = 7;       // pin for DMS enable switch
const int dmsDeadLED = 3;         // pin to signal DMS 'Dead' state
const int dmsAliveLED = A2;        // pin to signal DMS 'Alive' state
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

const bool HALTED = true;
const bool NOT_HALTED = false;

bool haltState = HALTED;                // variable for current halt state
int haltButtonState = 0;          // variable for reading halt button
int manualControlState = 0;       // variable for manual control
int joystickXValue = 0;           // variable for left/right
int joystickYValue = 0;           // variable for forward/reverse

int mappedX;
int mappedY;

enum RadioModes {
    NO_MOTION = 0,
    AUTONOMOUS = 1,
    HAND_CONTROL = 2,
};

unsigned char mode = 0;
const unsigned char SPEED_STOP = 128;

unsigned char motorLeft = SPEED_STOP;
unsigned char motorRight = SPEED_STOP;

const unsigned char RADIO_PACKET_LEN = 12;
unsigned char radiopacket[RADIO_PACKET_LEN];

void sendControlOutput(unsigned char m, unsigned char l, unsigned char r) {
    radiopacket[0] = 'm';
    radiopacket[1] = ':';
    radiopacket[2] = m;  // motion is disabled
    radiopacket[3] = ',';
    radiopacket[4] = 'l';
    radiopacket[5] = ':';
    radiopacket[6] = l;
    radiopacket[7] = ',';
    radiopacket[8] = 'r';
    radiopacket[9] = ':';
    radiopacket[10] = r;
    radiopacket[11] = ';';
    //String packet(radiopacket);
    //Serial.print("Sending "); Serial.println(packet);

    //target node Id, message as string or byte array, message length
    if (radio.sendWithRetry(RECEIVER, radiopacket, RADIO_PACKET_LEN)) {
        Serial.println("OK");
    }

    radio.receiveDone(); //put radio in RX mode
    Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
}

void setup() {
    Serial.print("\n");
  
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
    Serial.begin(SERIAL_BAUD);
    Serial.println("Arduino RFM69HCW Transmitter");
    // Hard Reset the RFM module
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, HIGH);
    delay(100);
    digitalWrite(RFM69_RST, LOW);
    delay(100);
  
    // Initialize radio
    radio.initialize(FREQUENCY,NODEID,NETWORKID);
    if (IS_RFM69HCW) {
        radio.setHighPower();    // Only for RFM69HCW & HW!
    }
    radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
    radio.encrypt(ENCRYPTKEY);
    pinMode(LED, OUTPUT);
    Serial.print("\nTransmitting at ");
    Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(" MHz");
}

void loop() {
    // First check if the DMS enable switch is on (red covered switch is in closed/down position):
    dmsEnableState = digitalRead(dmsEnablePin);

    // No motion unless dead man button is held down continuously.
    if (dmsEnableState == LOW) {
        //Serial.print("DMS is enabled.");
        digitalWrite(dmsDisabledLED, LOW);  // turn off blue LED
        digitalWrite(haltLED, LOW);  //  turn off yellow LED
        // Now check if DMS button is depressed:
        dmsButtonState = digitalRead(dmsLiveButton);
        // If DMS is enabled and DMS button is depressed, cancel 'Die' signal 
        // and send 'Alive' signal:
        if (dmsButtonState == HIGH) {
            digitalWrite(dmsDeadLED, LOW);  // turn off red LED
            digitalWrite(dmsAliveLED, HIGH);  // turn on green LED
            //Serial.print("\t DMS button is depressed (stay alive)\n");
            mode = AUTONOMOUS; // m1 allows autonomous motion
            motorLeft = SPEED_STOP; // the system should disregard this value in m1, we but we send 128 anyway
            motorRight = SPEED_STOP; // the system should disregard this value in m1, but we send 128 anyway
            sendControlOutput(mode, motorLeft, motorRight);
                     
        } else {
            // If DMS is enabled and DMS is not depressed, cancel 'Alive' 
            // signal and send 'Die' signal:
            digitalWrite(dmsAliveLED, LOW);  // turn off green LED
            digitalWrite(dmsDeadLED, HIGH);  // turn on red LED
            //Serial.print("\t DMS button is open (Die)\n");
            mode = NO_MOTION; // m1 allows autonomous motion
            motorLeft = SPEED_STOP; // the system should disregard this value in m0,  but we send 128 anyway
            motorRight = SPEED_STOP; // the system should disregard this value in m0, but we send 128 anyway
            sendControlOutput(mode, motorLeft, motorRight);
        }
    }

    // Motion is enabled and disabled by toggling the halt button (momentary press)
    if (dmsEnableState == HIGH) {
        // If the DMS is not enabled (red covered switch is uncovered and up), turn off DMS LEDs, turn on 'DMS OFF' indicator:
        digitalWrite(dmsDisabledLED, HIGH);  // turn on blue LED
        digitalWrite(dmsAliveLED, LOW);  // turn off green LED
        digitalWrite(dmsDeadLED, LOW);  //  turn off red LED
        //Serial.print("DMS is disabled.");
        // Check if halt button has been pushed, if so, toggle halt state:
        haltButtonState = digitalRead(haltButton);
        if (haltButtonState == HIGH) {
            haltState = !haltState;
            // For de-bouncing the push button.  don't hold it down for more
            // than half a second, push it once.
            delay(500);
        }

        digitalWrite(haltLED, haltState);
        if (haltState == HALTED) {
            //Serial.print("\t HALT!");
            mode = NO_MOTION; // m1 allows autonomous motion

            // The system should disregard this value in m1, we but we send 128 anyway.
            motorLeft = SPEED_STOP;
            motorRight = SPEED_STOP; // the system should disregard this value in m1, but we send 128 anyway
            sendControlOutput(mode, motorLeft, motorRight);
        }
        // Check if manual control switch is on (blue covered switch is up):
        manualControlState = digitalRead(manualControlPin);


        if (manualControlState == LOW) {
            //Serial.print("\t Manual-Control Disabled. Continuous-Autonomous Enabled");
            // Autonomous movement enabled (m:1) if not halted, but (m:0) if it is halted.
            mode = (unsigned char)(!haltState);
            // The system should disregard this value in this mode, we but we send 128 anyway
            motorLeft = SPEED_STOP;
            // The system should disregard this value in this mode, but we send 128 anyway
            motorRight = SPEED_STOP;
            sendControlOutput(mode, motorLeft, motorRight); 
        } else {
            //Serial.print("\t Manual Control Enabled.");
        }

        // IF DMS is off and manual control is on BUT the state is in HALT, disable motion
        if (manualControlState == HIGH and haltState == HALTED) {
            mode = NO_MOTION; // m1 allows autonomous motion
            motorLeft = SPEED_STOP; // the system should disregard this value in this mode, we but we send 128 anyway
            motorRight = SPEED_STOP; // the system should disregard this value in this mode, but we send 128 anyway
            sendControlOutput(mode, motorLeft, motorRight);       
        }

        // If DMS is off and manual control switch is on and not halted, read 
        // and report joystick values:
        if (manualControlState == HIGH and haltState == NOT_HALTED) {
            joystickXValue = analogRead(joystickXPin);
            joystickYValue = analogRead(joystickYPin);
            int mappedX = map(joystickXValue, 0, 1023, 0, 255);
            int mappedY = map(joystickYValue, 0, 1023, 0, 255);

            /*
            float mathX = (mappedX - 128);
            float mathY = (mappedY - 128);
            Serial.print("\tmappedX: ");
            Serial.print(mappedX);
            Serial.print("\tmappedY: ");
            Serial.print(mappedY);
            float toa = (mathY/mathX);
            Serial.print("\ttoa: ");
            Serial.print(toa);
            Serial.print("\ttheta: ");
            float theta = (atan(toa)*(57.2958));
            Serial.print(theta);
            */
            
            
            // 19 joystick zones
            // central deadzone
            if ((mappedX > 124 and mappedX < 132) and (mappedY > 124 and mappedY < 132)) {
                //Serial.print("\t Joystick is in deadzone \t");
                motorLeft = SPEED_STOP;
                motorRight = SPEED_STOP;
                //now send this data over radio
                mode = HAND_CONTROL;
                sendControlOutput(mode, motorLeft, motorRight);
            }
        }

        // Quadrant FR
        if ((mappedX >131) and (mappedY > 131)) {
            //Serial.print("\t Q=FR\t");
            float mathX = (mappedX - SPEED_STOP);
            float mathY = (mappedY - SPEED_STOP);

            // Corresponds to angles: 79-90, ie hard right
            if((mathY / mathX) < 0.194) {
                //Serial.print("\t going CW\t");
                motorLeft = mappedX;
                motorRight = map(mappedX, SPEED_STOP, 255, SPEED_STOP, 0);
            } else if ((mathY/mathX) < 0.424) {  //  corresponds to angles 67-79, ie above hard-right, pivot zone
                //Serial.print("\t about to go CW\t");
                motorLeft = map(mappedX, SPEED_STOP, 255, 128, 165);
                motorRight = SPEED_STOP;
            } else if ((mathY/mathX) < 1) {  //  corresponds to angles 45-67, tightest forward right arc
                //Serial.print("\t FR, about to pivot");
                motorLeft = map(mappedY, SPEED_STOP, 235, 128, 201);
                motorRight = map(mappedY, SPEED_STOP, 235, 128, 149);
            } else if ((mathY/mathX) < 2.47) {  // corresponds to angles 22-45, forward/right arc
                //Serial.print("\t FR");
                motorLeft = map(mappedY, 128, 254, 128, 218);
                motorRight = map(mappedY, 128, 255, 128, 192);
            } else {
                //Serial.print("\t FF-R");  //  corresponds to angles 0-22,
                motorLeft = map(mappedY, 128, 255, 128, 255);
                motorRight = map(mappedY, 128, 255, 128, 255);
            }

            //now send this data over radio 
            mode = HAND_CONTROL;
            sendControlOutput(mode, motorLeft, motorRight); 
        } // End forward right quadrant.
            
        // Quadrant BL
        if ((mappedX > 127) and (mappedY < 128)) {
            //Serial.print("\t Q=BL\t");
            float mathX = (mappedX - 128);
            float mathY = (128 -mappedY);
            if((mathY / mathX) < .194) {
                //Serial.print("\t going CW\t");
                motorLeft = mappedX;
                motorRight = map(mappedX, 128, 255, 128, 0);
            } else if ((mathY/mathX) < 0.424) {
                //Serial.print("\t about to go CW\t");
                motorLeft = map(mappedX, 128, 255, 128, 88);
                motorRight = 128;
            } else if ((mathY/mathX) < 1) {
                //Serial.print("\t BL-P, about to pivot");
                motorLeft = map(mappedY, 128, 36, 128, 54);
                motorRight = map(mappedY, 128, 36, 128, 106);
            } else if ((mathY/mathX) < 2.47) {
                //Serial.print("\t BL");
                motorLeft = map(mappedY, 128, 12, 128, 37);
                motorRight = map(mappedY, 128, 12, 128, 63);
            } else  {
                //Serial.print("\t BB-L");
                motorLeft = map(mappedY, 128, 0, 128, 0);
                motorRight = map(mappedY, 128, 0, 128, 0);
            }

            // Now send this data over radio
            mode = HAND_CONTROL;
            sendControlOutput(mode, motorLeft, motorRight);
        } // End Quadrant BL
                    
        // Quadrant BR
        if ((mappedX < 128) and (mappedY < 128)) {
            Serial.print("\t Q=BR\t");
            float mathX = (128 - mappedX);
            float mathY = (128 - mappedY);
              
            if((mathY / mathX) < 0.018) {
                //Serial.print("\t going CCW\t");
                motorLeft = mappedX;
                motorRight = map(mappedX, 128, 0, 128, 255);
            } else if ((mathY / mathX) < 0.424) {
                //Serial.print("\t about to go CCW\t");
                motorLeft = map(mappedX, 128, 31, 128, 88);
                motorRight = 128;
            } else if ((mathY / mathX) < 1.0) {
                //Serial.print("\t BR-P, about to pivot");
                motorLeft = map(mappedY, 128, 54, 128, 106);
                motorRight = map(mappedY, 128, 54, 128, 54);
            } else if ((mathY / mathX) < 2.47) {
                //Serial.print("\t BR");
                motorLeft = map(mappedY, 128, 27, 128, 63);
                motorRight = map(mappedY, 128, 27, 128, 37);
            } else {
                //Serial.print("\t BB-R");
                motorLeft = map(mappedY, 128, 0, 128, 0);
                motorRight = map(mappedY, 128, 0, 128, 0);
            }
            // Now send this data over radio::
            mode = HAND_CONTROL;
            sendControlOutput(mode, motorLeft, motorRight);
        } // End Quadrant BR.

        // Quadrant FL
        if ((mappedX < 128) and (mappedY > 127)) {
              //Serial.print("\t Q=FL\t");
              float mathX = (128 - mappedX);
              float mathY = (mappedY - 128); 
              if((mathY/mathX) < 0.199) {
                  //Serial.print("\t going CCW\t");
                  motorLeft = mappedX;
                  motorRight = map(mappedX, 128, 0, 128, 255);
              } else if ((mathY/mathX) < 0.424) {
                  //Serial.print("\t about to go CW\t");
                  motorLeft = 128;
                  motorRight = map(mappedX, 128, 0, 128, 165);
              } else if ((mathY/mathX) < 1.0) {
                  //Serial.print("\t FR, about to pivot");
                  motorLeft = map(mappedY, 128, 212, 128, 149);
                  motorRight = map(mappedY, 128, 212, 128, 201);
              } else if ((mathY / mathX) < 2.47) {
                  //Serial.print("\t FR");
                  motorLeft = map(mappedY, 128, 244, 128, 192);
                  motorRight = map(mappedY, 128, 244, 128, 218);
              } else {
                  //Serial.print("\t FF-R");
                  motorLeft = map(mappedY, 128, 255, 128, 255);
                  motorRight = map(mappedY, 128, 255, 128, 255);
             }

            //now send this data over radio 
            mode = HAND_CONTROL;
            sendControlOutput(mode, motorLeft, motorRight); 
        } // End Quadrant FL.
    }
}


