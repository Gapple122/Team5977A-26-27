/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       prane                                                     */
/*    Created:      9/27/2026, 4:08:57 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath>
#include <cstring>

using namespace vex;

brain Brain;
controller Controller1 = controller(primary);

// Higher = gentler turning near center. 1.0 = linear, 2.0 = squared, 3.0 = cubed.
const double TURN_CURVE = 4;
// Overall turn speed limit. 1.0 = full speed, 0.5 = half speed at full stick.
const double TURN_SCALE = 0.8;
const int DEADBAND = 5; // ignore tiny stick drift

// Maps -100..100 input to -100..100 output along a power curve, keeping the sign.
double curve(int input, double exponent) {
    if (abs(input) < DEADBAND) return 0;
    double normalized = abs(input) / 100.0;           // 0.0 to 1.0
    double output = pow(normalized, exponent) * 100;  // curved 0 to 100
    return (input < 0) ? -output : output;
}

/*----------------------------------------------------------------------------*/
/*  Motors (created at runtime so their ports can be changed from the screen) */
/*  Index 0,1 = left side.  2,3 = right side.  4,5 = 5.5W aux motors.         */
/*                                                                            */
/*  motorPorts[] is the single source of truth for which port each motor      */
/*  uses. Moving a motor on the screen updates it and rebuilds the motor on   */
/*  the new port, so the drive loop follows immediately.                      */
/*----------------------------------------------------------------------------*/

const int NUM_MOTORS = 8;
const int MIN_PORT = 1;
const int MAX_PORT = 21;

// Motor indexes: 0,1 = left drive   2,3 = right drive   4,5 = 5.5W aux motors
const int AUX_A = 4;
const int AUX_B = 5;
const int B1_MOTOR = 6;
const int B2_MOTOR = 7;
const int CASCADE_MOTOR = B1_MOTOR;

const char* motorNames[NUM_MOTORS] = { "L Front", "L Back", "R Front", "R Back", "Aux 1", "Aux 2", "B1", "B2" };
const char* motorShort[NUM_MOTORS] = { "LF", "LB", "RF", "RB", "A1", "A2", "B1", "B2" };
const char* motorDescriptions[NUM_MOTORS] = { "Drive motor", "Drive motor", "Drive motor", "Drive motor", "Claw", "Claw", "Cascade", "Intake" };
// Starting ports (1-21) and directions. Changed live from the Brain screen.
int  motorPorts[NUM_MOTORS]    = { 3, 2, 4, 1, 5, 6, 7, 8 };
bool motorReversed[NUM_MOTORS] = { false, true, true, false, false, false, false, false };

// Drive motors use 6:1 cartridges. The 5.5W motors have a fixed 200 rpm gearing,
// which the SDK treats as 18:1.
gearSetting motorGears[NUM_MOTORS] = {
    gearSetting::ratio6_1, gearSetting::ratio6_1,
    gearSetting::ratio6_1, gearSetting::ratio6_1,
    gearSetting::ratio18_1, gearSetting::ratio18_1,
    gearSetting::ratio18_1, gearSetting::ratio18_1
};

motor* motors[NUM_MOTORS] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

// Protects the motors[] pointers: the drive loop and the screen thread both use them.
vex::mutex motorLock;

// Live controller input, written by the drive loop and read by the screen (for the
// CONTROL panel on the motor pages). motorCommand[] = power sent to each motor, -100..100.
volatile int  inputFwd  = 0;     // left stick, vertical (Axis3)
volatile int  inputTurn = 0;     // right stick, horizontal (Axis1)
volatile bool inputL1   = false; // top left trigger
volatile bool inputL2   = false; // bottom left trigger
volatile bool inputR1   = false; // top right trigger
volatile bool inputR2   = false; // bottom right trigger
volatile int  motorCommand[NUM_MOTORS];

// Touchscreen controller emulation. A held touch acts like a held controller input.
volatile int touchFwd = 0;
volatile int touchTurn = 0;
volatile bool touchStickLeft = false;
volatile bool touchStickRight = false;
volatile bool touchL1 = false;
volatile bool touchL2 = false;
volatile bool touchR1 = false;
volatile bool touchR2 = false;
volatile bool touchA = false, touchB = false, touchX = false, touchY = false;
volatile bool touchUp = false, touchDown = false, touchLeft = false, touchRight = false;

// A motor-page speed slider is a momentary command: releasing it always stops it.
volatile bool touchMotorActive = false;
volatile int touchMotorPower = 0;

enum TouchControl {
    TOUCH_NONE, TOUCH_LEFT_STICK, TOUCH_RIGHT_STICK, TOUCH_SLIDER,
    TOUCH_L1, TOUCH_L2, TOUCH_R1, TOUCH_R2,
    TOUCH_A, TOUCH_B, TOUCH_X, TOUCH_Y,
    TOUCH_UP, TOUCH_DOWN, TOUCH_LEFT, TOUCH_RIGHT
};
TouchControl activeTouchControl = TOUCH_NONE;

// (Re)creates motor i from its current port/reversed settings. Caller must hold motorLock
// (or be running before the screen thread starts).
void rebuildMotor(int i) {
    if (motors[i] != nullptr) {
        motors[i]->stop();
        delete motors[i];
    }
    // SDK port indexes are 0-based: PORT1 == 0
    motors[i] = new motor(motorPorts[i] - 1, motorGears[i], motorReversed[i]);
    motors[i]->setStopping(brake);
}

// Which motor (0-3) is assigned to this port, or -1 if none.
int motorAtPort(int port) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motorPorts[i] == port) return i;
    }
    return -1;
}

// Move motor i to a port. If another motor is already there, the two swap.
void moveMotorToPort(int i, int port) {
    if (port < MIN_PORT || port > MAX_PORT || port == motorPorts[i]) return;
    int other = motorAtPort(port);

    motorLock.lock();
    if (other >= 0 && other != i) {
        motorPorts[other] = motorPorts[i];
        motorPorts[i] = port;
        rebuildMotor(other);
        rebuildMotor(i);
    } else {
        motorPorts[i] = port;
        rebuildMotor(i);
    }
    motorLock.unlock();
}

/*----------------------------------------------------------------------------*/
/*  Smart-port probes: one generic device handle per port, so the screen can  */
/*  show what is physically plugged in (even if the code doesn't use it).     */
/*----------------------------------------------------------------------------*/

device* probes[MAX_PORT + 1] = { nullptr };   // indexed 1..21

struct PortData {
    bool installed;
    int type;          // V5 device type id, 0 = nothing
};
PortData portSnap[MAX_PORT + 1];

// Full name, used on the port diagnostic screen.
const char* deviceTypeName(int type) {
    switch (type) {
        case 0:  return "None";
        case 2:  return "Motor";
        case 3:  return "LED";
        case 4:  return "Rotation Sensor";
        case 6:  return "Inertial";
        case 7:  return "Distance";
        case 8:  return "Radio";
        case 11: return "Vision";
        case 12: return "3-Wire Expander";
        case 16: return "Optical";
        case 17: return "Electromagnet";
        case 20: return "GPS";
        case 26: return "AI Camera";
        case 27: return "Light Tower";
        case 28: return "Arm";
        case 29: return "AI Vision";
        case 30: return "Pneumatics";
        default: return "Unknown device";
    }
}

// 3-letter code, used on the grid tiles (must stay short so it fits the tile).
const char* deviceShortName(int type) {
    switch (type) {
        case 2:  return "mtr";
        case 4:  return "rot";
        case 6:  return "imu";
        case 7:  return "dst";
        case 11: return "vis";
        case 12: return "adi";
        case 16: return "opt";
        case 20: return "gps";
        case 26: return "cam";
        case 29: return "ai";
        case 30: return "pnu";
        default: return "dev";
    }
}

/*----------------------------------------------------------------------------*/
/*  3-wire labels                                                             */
/*  3-wire devices can't be identified from the Brain like smart-port devices, */
/*  so these are just labels. Edit them to match your robot. Keep each name   */
/*  to 8 characters or less (longer names are cut off on screen).             */
/*----------------------------------------------------------------------------*/

const int NUM_3WIRE = 8;
const char* threeWireNames[NUM_3WIRE] = { "A", "B", "C", "D", "E", "F", "G", "H" };
const char* threeWireDevices[NUM_3WIRE] = {
    "Not set", "Not set", "Not set", "Not set",
    "Not set", "Not set", "Not set", "Not set"
};

/*----------------------------------------------------------------------------*/
/*  Screen layout (480 x 240)                                                 */
/*                                                                            */
/*   x 0-103   : 3-wire list (header, 5 rows, scroll buttons)                 */
/*   x 105     : divider line                                                 */
/*   x 110-476 : smart-port grid (title + SWITCH button, hint, 7x3 tiles,     */
/*               legend)                                                      */
/*  Font widths used for fitting text: mono15 = 9 px, mono20 = 12 px.         */
/*----------------------------------------------------------------------------*/

// 3-wire column
const int TW_TOP = 24;
const int TW_ROW_H = 38;
const int TW_VISIBLE = 5;                 // rows 24..214
const int TW_ARROW_Y = 216;
const int TW_ARROW_H = 22;
const int TW_UP_X = 2;
const int TW_DOWN_X = 54;
const int TW_ARROW_W = 48;
const int TW_NAME_CHARS = 8;              // max device-name characters per row

const int DIVIDER_X = 105;
const int PANEL_X = 114;

// Port assignment and direction controls
const int SWITCH_X = 278;
const int SWITCH_Y = 4;
const int SWITCH_W = 90;
const int SWITCH_H = 26;
const int REVERSE_X = 374;
const int REVERSE_W = 100;

// Smart-port grid: 7 columns x 3 rows = 21 tiles (y 52..220)
const int TILE_COLS = 7;
const int TILE_ROWS = 3;
const int TILE_W = 52;
const int TILE_H = 56;
const int TILE_X0 = 112;
const int TILE_Y0 = 52;

// CONTROL panel on motor pages (right side, x 280..472, y 3..198)
const int CTRL_X = 280;

// BACK button on diagnostic screens
const int BACK_X = 370;
const int BACK_Y = 206;
const int BACK_W = 100;
const int BACK_H = 28;

enum ScreenPage {
    HOME_PAGE,
    MOTOR_DIAGNOSTIC_PAGE,
    PORT_DIAGNOSTIC_PAGE
};

ScreenPage currentPage = HOME_PAGE;
int diagnosticMotor = -1;
int diagnosticPort = -1;
int selectedMotor = -1;        // motor picked up while SWITCH is on
int selectedTw = -1;           // highlighted 3-wire row
int threeWireScroll = 0;
bool switchPortsMode = false;
bool reversePortsMode = false;
bool pendingMotorReversed[NUM_MOTORS];

// Snapshot of motor readings, so drawing never holds the lock the drive loop needs.
struct MotorData {
    bool installed;
    double temp, rpm, amps, volts, torque, watts, eff, pos;
};
MotorData snap[NUM_MOTORS];

void takeSnapshot() {
    motorLock.lock();
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor& m = *motors[i];
        snap[i].installed = m.installed();
        snap[i].temp   = m.temperature(celsius);
        snap[i].rpm    = m.velocity(rpm);
        snap[i].amps   = m.current(amp);
        snap[i].volts  = m.voltage(volt);
        snap[i].torque = m.torque(Nm);
        snap[i].watts  = m.power(watt);
        snap[i].eff    = m.efficiency(percent);
        snap[i].pos    = m.position(degrees);
    }
    motorLock.unlock();

    for (int p = MIN_PORT; p <= MAX_PORT; p++) {
        portSnap[p].installed = probes[p]->installed();
        portSnap[p].type = portSnap[p].installed ? (int)probes[p]->type() : 0;
    }
}

// Green = cool, yellow = warm, red = hot (V5 motors throttle around 55C+).
color tempColor(double tempC) {
    if (tempC < 45) return color::green;
    if (tempC < 55) return color::yellow;
    return color::red;
}

bool inRect(int px, int py, int x, int y, int w, int h) {
    return px >= x && px < x + w && py >= y && py < y + h;
}

// Button with the label centered. Set the font first; charW/charH must match it.
void drawButton(int x, int y, int w, int h, const char* label, color fill, int charW, int charH) {
    Brain.Screen.setPenWidth(1);
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setFillColor(fill);
    Brain.Screen.drawRectangle(x, y, w, h);
    int textW = (int)strlen(label) * charW;
    Brain.Screen.printAt(x + (w - textW) / 2, y + h / 2 + charH / 2 - 2, true, "%s", label);
}

// Prints at most maxChars characters of s, so text can never spill past its box.
void printFit(int x, int y, const char* s, int maxChars) {
    char buf[24];
    int n = (int)strlen(s);
    if (n > maxChars) n = maxChars;
    if (n > 23) n = 23;
    memcpy(buf, s, n);
    buf[n] = '\0';
    Brain.Screen.printAt(x, y, true, "%s", buf);
}

void drawThreeWireList() {
    int maxScroll = NUM_3WIRE - TW_VISIBLE;
    int last = threeWireScroll + TW_VISIBLE - 1;
    if (last > NUM_3WIRE - 1) last = NUM_3WIRE - 1;

    // Header: title on the left, visible range (e.g. "A-E") on the right
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.printAt(4, 17, true, "3-WIRE");
    Brain.Screen.setPenColor(color(150, 150, 150));
    Brain.Screen.printAt(68, 17, true, "%s-%s", threeWireNames[threeWireScroll], threeWireNames[last]);

    // Rows
    for (int row = 0; row < TW_VISIBLE; row++) {
        int i = threeWireScroll + row;
        if (i >= NUM_3WIRE) break;

        int y = TW_TOP + row * TW_ROW_H;
        bool selected = (selectedTw == i);

        Brain.Screen.setPenWidth(selected ? 2 : 1);
        Brain.Screen.setPenColor(selected ? color::yellow : color(80, 80, 80));
        Brain.Screen.setFillColor(selected ? color(70, 70, 20) : color(35, 35, 35));
        Brain.Screen.drawRectangle(2, y, 100, TW_ROW_H - 2);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.setFont(mono20);
        Brain.Screen.printAt(8, y + 26, true, "%s", threeWireNames[i]);

        Brain.Screen.setFont(mono15);
        Brain.Screen.setPenColor(strcmp(threeWireDevices[i], "Not set") == 0 ? color(130, 130, 130) : color::white);
        printFit(26, y + 23, threeWireDevices[i], TW_NAME_CHARS);
    }
    Brain.Screen.setPenWidth(1);

    // Scroll buttons (dimmed when they can't scroll any further)
    Brain.Screen.setFont(mono15);
    drawButton(TW_UP_X, TW_ARROW_Y, TW_ARROW_W, TW_ARROW_H, "^",
               threeWireScroll > 0 ? color(70, 70, 70) : color(25, 25, 25), 9, 15);
    drawButton(TW_DOWN_X, TW_ARROW_Y, TW_ARROW_W, TW_ARROW_H, "v",
               threeWireScroll < maxScroll ? color(70, 70, 70) : color(25, 25, 25), 9, 15);
}

void drawSmartPortGrid() {
    drawThreeWireList();

    // Divider between the two areas
    Brain.Screen.setPenWidth(1);
    Brain.Screen.setPenColor(color(60, 60, 60));
    Brain.Screen.drawLine(DIVIDER_X, 0, DIVIDER_X, 240);

    // Title
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.setFont(mono20);
    Brain.Screen.printAt(PANEL_X, 22, true, "SMART PORTS");

    // Port tools: SWITCH/DONE swaps motor assignments; REVERSE/DONE stages
    // and confirms per-motor direction changes.
    Brain.Screen.setFont(mono15);
    drawButton(SWITCH_X, SWITCH_Y, SWITCH_W, SWITCH_H,
               switchPortsMode ? "DONE" : "SWITCH",
               switchPortsMode ? color(0, 110, 0) :
               reversePortsMode ? color(25, 25, 25) : color(60, 60, 60), 9, 15);
    drawButton(REVERSE_X, SWITCH_Y, REVERSE_W, SWITCH_H,
               reversePortsMode ? "DONE" : "REVERSE",
               reversePortsMode ? color(190, 90, 0) :
               switchPortsMode ? color(25, 25, 25) : color(60, 60, 60), 9, 15);

    // Hint line (sits between the header and the grid)
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono15);
    if (reversePortsMode) {
        Brain.Screen.setPenColor(color(255, 175, 70));
        Brain.Screen.printAt(PANEL_X, 46, true, "Tap motor ports to toggle direction");
    } else if (switchPortsMode && selectedMotor >= 0) {
        Brain.Screen.setPenColor(color::yellow);
        Brain.Screen.printAt(PANEL_X, 46, true, "Move %s: tap new port", motorNames[selectedMotor]);
    } else if (switchPortsMode) {
        Brain.Screen.setPenColor(color::yellow);
        Brain.Screen.printAt(PANEL_X, 46, true, "Tap a motor, then its new port");
    } else {
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X, 46, true, "Tap a port for diagnostics");
    }

    // Tiles
    for (int p = MIN_PORT; p <= MAX_PORT; p++) {
        int col = (p - 1) % TILE_COLS;
        int row = (p - 1) / TILE_COLS;
        int x = TILE_X0 + col * TILE_W;
        int y = TILE_Y0 + row * TILE_H;

        int m = motorAtPort(p);

        color fill = color(40, 40, 40);          // empty
        const char* label = "";

        if (m >= 0) {
            bool reversed = reversePortsMode ? pendingMotorReversed[m] : motorReversed[m];
            if (!portSnap[p].installed) fill = color(140, 0, 0);
            else fill = reversed ? color(210, 105, 0) : color(0, 110, 0);
            label = motorShort[m];               // LF / LB / RF / RB
        } else if (portSnap[p].installed) {
            fill = color(0, 70, 140);            // plugged in, not used by the code
            label = deviceShortName(portSnap[p].type);
        }

        bool held = (switchPortsMode && m >= 0 && m == selectedMotor);
        bool reverseChanged = (reversePortsMode && m >= 0 &&
                               pendingMotorReversed[m] != motorReversed[m]);

        Brain.Screen.setPenWidth((held || reverseChanged) ? 3 : 1);
        Brain.Screen.setPenColor(held ? color::yellow :
                                 reverseChanged ? color(255, 175, 70) : color(90, 90, 90));
        Brain.Screen.setFillColor(fill);
        Brain.Screen.drawRectangle(x, y, TILE_W - 2, TILE_H - 2);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.setFont(mono20);
        Brain.Screen.printAt(x + 5, y + 24, true, "%d", p);

        if (label[0] != '\0') {
            Brain.Screen.setFont(mono15);
            Brain.Screen.printAt(x + 5, y + 46, true, "%s", label);
        }
    }
    Brain.Screen.setPenWidth(1);

    // Legend
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color(170, 170, 170));
    Brain.Screen.printAt(PANEL_X, 236, true, "green=ok orange=rev red=miss blue=other");
}

// Controller controls are drawn as real touch targets. Cyan outlines indicate
// controls used by this motor group; a filled tile indicates an active input.
void drawControlButton(int x, int y, int w, int h, const char* label,
                       bool used, bool pressed) {
    Brain.Screen.setPenWidth(used ? 2 : 1);
    Brain.Screen.setPenColor(used ? color::cyan : color(95, 95, 95));
    Brain.Screen.setFillColor(pressed ? color(0, 125, 80) : color(30, 30, 30));
    Brain.Screen.drawRectangle(x, y, w, h);
    Brain.Screen.setFont(mono12);
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.printAt(x + 3, y + h - 3, true, "%s", label);
    Brain.Screen.setPenWidth(1);
}

// Joystick graphic. dx/dy are -100..100; dy positive means up.
void drawStick(int cx, int cy, bool used, int dx, int dy, bool touched) {
    color rim = used ? color(235, 190, 55) : color(105, 118, 128);
    Brain.Screen.setPenWidth(1);
    Brain.Screen.setPenColor(rim);
    Brain.Screen.setFillColor(color(18, 24, 30));
    Brain.Screen.drawCircle(cx, cy, 14);
    Brain.Screen.setPenColor(color(56, 68, 77));
    Brain.Screen.drawCircle(cx, cy, 10);
    Brain.Screen.setPenColor(touched ? color::green : rim);
    Brain.Screen.setFillColor(touched ? color::green : rim);
    Brain.Screen.drawCircle(cx + dx * 9 / 100, cy - dy * 9 / 100, 4);
}

// Draw the controller infographic and a centered, momentary motor-speed bar.
void drawControlPanel(int idx) {
    bool aux = (idx == AUX_A || idx == AUX_B);
    bool b1 = (idx == B1_MOTOR);
    bool b2 = (idx == B2_MOTOR);
    int fwd = inputFwd;
    int turn = inputTurn;
    int cmd = motorCommand[idx];

    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.printAt(CTRL_X, 16, true, "CONTROL");

    // Shoulder triggers: right pair operate the opposing claw motors.
    drawControlButton(286, 25, 34, 18, "L1", b2, inputL1 || touchL1);
    drawControlButton(323, 25, 34, 18, "L2", b2, inputL2 || touchL2);
    drawControlButton(397, 25, 34, 18, "R1", aux, inputR1 || touchR1);
    drawControlButton(434, 25, 34, 18, "R2", aux, inputR2 || touchR2);

    // Compact controller face: each group has its own even-width lane.
    Brain.Screen.setPenWidth(1);
    Brain.Screen.setPenColor(color(105, 120, 132));
    Brain.Screen.setFillColor(color(27, 35, 43));
    Brain.Screen.drawRectangle(277, 49, 198, 88);
    Brain.Screen.setPenColor(color(52, 65, 75));
    Brain.Screen.drawLine(336, 56, 336, 130);
    Brain.Screen.drawLine(420, 56, 420, 130);

    drawStick(354, 86, !aux && !b1 && !b2, 0,
              touchStickLeft ? touchFwd : fwd, touchStickLeft);
    drawStick(402, 86, !aux && !b1 && !b2, touchStickRight ? touchTurn : turn,
              0, touchStickRight);

    // D-pad and ABXY are arranged as compact, balanced clusters.
    drawControlButton(297, 59, 17, 17, "^", false, touchUp);
    drawControlButton(297, 96, 17, 17, "v", b1,
                      Controller1.ButtonDown.pressing() || touchDown);
    drawControlButton(279, 78, 17, 17, "<", false, touchLeft);
    drawControlButton(315, 78, 17, 17, ">", b1,
                      Controller1.ButtonRight.pressing() || touchRight);
    drawControlButton(440, 58, 17, 17, "Y", false, touchY);
    drawControlButton(458, 78, 16, 17, "B", false, touchB);
    drawControlButton(440, 98, 17, 17, "A", false, touchA);
    drawControlButton(422, 78, 16, 17, "X", false, touchX);

    Brain.Screen.setFont(mono12);
    Brain.Screen.setPenColor(color(145, 158, 168));
    Brain.Screen.printAt(291, 132, true, "DPAD");
    Brain.Screen.printAt(341, 132, true, "MOVE");
    Brain.Screen.printAt(389, 132, true, "TURN");
    Brain.Screen.printAt(433, 132, true, "ABXY");

    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono12);
    Brain.Screen.setPenColor(color(190, 205, 215));
    if (b2) {
        Brain.Screen.printAt(CTRL_X, 151, true, "L1/L2  INTAKE IN / OUT");
    } else if (b1) {
        Brain.Screen.printAt(CTRL_X, 151, true, "D-PAD RIGHT/DOWN  B1");
    } else if (idx == AUX_A || idx == AUX_B) {
        Brain.Screen.printAt(CTRL_X, 151, true, "R1/R2  CLAW IN / OUT");
    } else {
        Brain.Screen.printAt(CTRL_X, 151, true, "LEFT STICK DRIVE  RIGHT TURN");
    }

    // Speed slider is centered at zero. Drag either direction to run; release to stop.
    Brain.Screen.setPenColor(color(170, 170, 170));
    Brain.Screen.printAt(CTRL_X, 168, true, "MOTOR SPEED");
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.printAt(CTRL_X + 144, 168, true, "%+4d%%", cmd);

    const int barX = CTRL_X, barY = 176, barW = 192, barH = 20;
    const int mid = barX + barW / 2;
    int fill = (cmd < 0 ? -cmd : cmd) * (barW / 2 - 1) / 100;
    Brain.Screen.setPenWidth(1);
    Brain.Screen.setPenColor(color(90, 90, 90));
    Brain.Screen.setFillColor(color(30, 30, 30));
    Brain.Screen.drawRectangle(barX, barY, barW, barH);
    if (fill > 0) {
        color c = (cmd > 0) ? color(0, 160, 0) : color(210, 110, 0);
        Brain.Screen.setPenColor(c);
        Brain.Screen.setFillColor(c);
        if (cmd > 0) Brain.Screen.drawRectangle(mid + 1, barY + 1, fill, barH - 2);
        else Brain.Screen.drawRectangle(mid - fill, barY + 1, fill, barH - 2);
    }
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.drawLine(mid, barY - 2, mid, barY + barH + 1);
}

void drawBackButton() {
    Brain.Screen.setFont(mono15);
    drawButton(BACK_X, BACK_Y, BACK_W, BACK_H, "BACK", color(55, 55, 55), 9, 15);
}

void drawMotorPage(int idx) {
    const MotorData& d = snap[idx];

    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono20);

    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.printAt(10, 23, true, "%s  (port %d)", motorNames[idx], motorPorts[idx]);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color(180, 180, 180));
    Brain.Screen.printAt(10, 43, true, "Desc: %s", motorDescriptions[idx]);

    if (!d.installed) {
        Brain.Screen.setFont(mono20);
        Brain.Screen.setPenColor(color::red);
        Brain.Screen.printAt(10, 78, true, "NOT DETECTED");
        Brain.Screen.setFont(mono15);
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(10, 102, true, "Check the cable and port.");
    } else {
        Brain.Screen.setFont(mono15);
        Brain.Screen.setPenColor(tempColor(d.temp));
        Brain.Screen.printAt(10,  68, true, "Temp:       %3.0f C", d.temp);
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(10,  86, true, "Velocity:   %5.0f rpm", d.rpm);
        Brain.Screen.printAt(10, 104, true, "Current:    %5.2f A",   d.amps);
        Brain.Screen.printAt(10, 122, true, "Voltage:    %5.2f V",   d.volts);
        Brain.Screen.printAt(10, 140, true, "Torque:     %5.2f Nm",  d.torque);
        Brain.Screen.printAt(10, 158, true, "Power:      %5.1f W",   d.watts);
        Brain.Screen.printAt(10, 176, true, "Efficiency: %5.0f %%",  d.eff);
        Brain.Screen.printAt(10, 194, true, "Position:   %5.0f deg", d.pos);
    }

    drawControlPanel(idx);
    drawBackButton();
}

void drawPortDiagnosticPage(int port) {
    // A port with a motor assigned uses the motor page (it draws its own title).
    int m = motorAtPort(port);
    if (m >= 0) {
        drawMotorPage(m);
        return;
    }

    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono20);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.printAt(10, 24, true, "Port %d", port);

    if (portSnap[port].installed) {
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(10, 62, true, "%s", deviceTypeName(portSnap[port].type));

        Brain.Screen.setFont(mono15);
        Brain.Screen.setPenColor(color(170, 170, 170));
        Brain.Screen.printAt(10, 90, true, "Type ID: %d", portSnap[port].type);
        Brain.Screen.printAt(10, 112, true, "Not used by the robot code.");
    } else {
        Brain.Screen.setPenColor(color::red);
        Brain.Screen.printAt(10, 62, true, "NO DEVICE DETECTED");
    }

    drawBackButton();
}

void handleSmartGridTouch(int x, int y) {
    if (inRect(x, y, SWITCH_X, SWITCH_Y, SWITCH_W, SWITCH_H)) {
        if (reversePortsMode) return;
        switchPortsMode = !switchPortsMode;
        selectedMotor = -1;
        return;
    }
    if (inRect(x, y, REVERSE_X, SWITCH_Y, REVERSE_W, SWITCH_H)) {
        if (switchPortsMode) return;
        if (!reversePortsMode) {
            for (int i = 0; i < NUM_MOTORS; i++)
                pendingMotorReversed[i] = motorReversed[i];
            reversePortsMode = true;
        } else {
            motorLock.lock();
            for (int i = 0; i < NUM_MOTORS; i++) {
                if (pendingMotorReversed[i] != motorReversed[i]) {
                    motorReversed[i] = pendingMotorReversed[i];
                    rebuildMotor(i);
                }
            }
            motorLock.unlock();
            reversePortsMode = false;
        }
        return;
    }

    if (!inRect(x, y, TILE_X0, TILE_Y0, TILE_COLS * TILE_W, TILE_ROWS * TILE_H)) return;

    int col = (x - TILE_X0) / TILE_W;
    int row = (y - TILE_Y0) / TILE_H;
    int port = row * TILE_COLS + col + 1;
    if (port < MIN_PORT || port > MAX_PORT) return;

    int m = motorAtPort(port);

    if (reversePortsMode) {
        if (m >= 0) pendingMotorReversed[m] = !pendingMotorReversed[m];
        return;
    }

    if (switchPortsMode) {
        if (selectedMotor < 0) {
            selectedMotor = m;                       // -1 if the port has no motor
        } else if (port == motorPorts[selectedMotor]) {
            selectedMotor = -1;                      // tapped it again: cancel
        } else {
            moveMotorToPort(selectedMotor, port);
            selectedMotor = -1;
        }
    } else {
        diagnosticPort = port;
        diagnosticMotor = m;
        currentPage = (m >= 0) ? MOTOR_DIAGNOSTIC_PAGE : PORT_DIAGNOSTIC_PAGE;
    }
}

void handleThreeWireTouch(int x, int y) {
    int maxScroll = NUM_3WIRE - TW_VISIBLE;

    if (inRect(x, y, TW_UP_X, TW_ARROW_Y, TW_ARROW_W, TW_ARROW_H)) {
        if (threeWireScroll > 0) threeWireScroll--;
        return;
    }
    if (inRect(x, y, TW_DOWN_X, TW_ARROW_Y, TW_ARROW_W, TW_ARROW_H)) {
        if (threeWireScroll < maxScroll) threeWireScroll++;
        return;
    }

    if (!inRect(x, y, 0, TW_TOP, DIVIDER_X, TW_VISIBLE * TW_ROW_H)) return;

    int index = threeWireScroll + (y - TW_TOP) / TW_ROW_H;
    if (index >= 0 && index < NUM_3WIRE) {
        selectedTw = (selectedTw == index) ? -1 : index;
    }
}

// Map a held finger to a virtual joystick or button. The controller graphic is
// on the motor diagnostic page, so touches here never interfere with the home grid.
int clampPercent(int value) {
    if (value > 100) return 100;
    if (value < -100) return -100;
    return value;
}

void clearTouchControl() {
    touchFwd = 0;
    touchTurn = 0;
    touchStickLeft = false;
    touchStickRight = false;
    touchL1 = touchL2 = touchR1 = touchR2 = false;
    touchA = touchB = touchX = touchY = false;
    touchUp = touchDown = touchLeft = touchRight = false;
    touchMotorPower = 0;
    touchMotorActive = false;
    if (diagnosticMotor >= 0 && diagnosticMotor < NUM_MOTORS)
        motorCommand[diagnosticMotor] = 0;
    activeTouchControl = TOUCH_NONE;
}

void updateTouchControl(int x, int y) {
    switch (activeTouchControl) {
        case TOUCH_LEFT_STICK:
            touchFwd = clampPercent((86 - y) * 5);
            break;
        case TOUCH_RIGHT_STICK:
            touchTurn = clampPercent((x - 402) * 5);
            break;
        case TOUCH_SLIDER:
            touchMotorPower = clampPercent((x - 376) * 100 / 96);
            touchMotorActive = true;
            break;
        default: break;
    }
}

void beginTouchControl(int x, int y) {
    if (inRect(x, y, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        currentPage = HOME_PAGE;
        diagnosticMotor = -1;
        diagnosticPort = -1;
        clearTouchControl();
        return;
    }

    // Right triggers operate the claw pair; left triggers are left available.
    if (inRect(x, y, 286, 25, 34, 18)) activeTouchControl = TOUCH_L1;
    else if (inRect(x, y, 323, 25, 34, 18)) activeTouchControl = TOUCH_L2;
    else if (inRect(x, y, 397, 25, 34, 18)) activeTouchControl = TOUCH_R1;
    else if (inRect(x, y, 434, 25, 34, 18)) activeTouchControl = TOUCH_R2;
    // D-pad and ABXY are checked before the nearby stick hit areas.
    else if (inRect(x, y, 297, 59, 17, 17)) activeTouchControl = TOUCH_UP;
    else if (inRect(x, y, 297, 96, 17, 17)) activeTouchControl = TOUCH_DOWN;
    else if (inRect(x, y, 279, 78, 17, 17)) activeTouchControl = TOUCH_LEFT;
    else if (inRect(x, y, 315, 78, 17, 17)) activeTouchControl = TOUCH_RIGHT;
    else if (inRect(x, y, 440, 58, 17, 17)) activeTouchControl = TOUCH_Y;
    else if (inRect(x, y, 458, 78, 16, 17)) activeTouchControl = TOUCH_B;
    else if (inRect(x, y, 440, 98, 17, 17)) activeTouchControl = TOUCH_A;
    else if (inRect(x, y, 422, 78, 16, 17)) activeTouchControl = TOUCH_X;
    // Compact joystick hit areas, separated from the D-pad and ABXY buttons.
    else if ((x - 354) * (x - 354) + (y - 86) * (y - 86) <= 19 * 19)
        activeTouchControl = TOUCH_LEFT_STICK;
    else if ((x - 402) * (x - 402) + (y - 86) * (y - 86) <= 19 * 19)
        activeTouchControl = TOUCH_RIGHT_STICK;
    // Momentary motor-speed slider.
    else if (inRect(x, y, CTRL_X, 176, 192, 20)) activeTouchControl = TOUCH_SLIDER;

    switch (activeTouchControl) {
        case TOUCH_L1: touchL1 = true; break;
        case TOUCH_L2: touchL2 = true; break;
        case TOUCH_R1: touchR1 = true; break;
        case TOUCH_R2: touchR2 = true; break;
        case TOUCH_A: touchA = true; break;
        case TOUCH_B: touchB = true; break;
        case TOUCH_X: touchX = true; break;
        case TOUCH_Y: touchY = true; break;
        case TOUCH_UP: touchUp = true; break;
        case TOUCH_DOWN: touchDown = true; break;
        case TOUCH_LEFT: touchLeft = true; break;
        case TOUCH_RIGHT: touchRight = true; break;
        case TOUCH_SLIDER: touchMotorActive = true; break;
        default: break;
    }
    updateTouchControl(x, y);
}

void handleTouch(int x, int y) {
    if (currentPage == HOME_PAGE) {
        if (x < DIVIDER_X) handleThreeWireTouch(x, y);
        else               handleSmartGridTouch(x, y);
        return;
    }
    beginTouchControl(x, y);
}

// Runs on its own thread so drawing never slows the drive loop.
int screenTask() {
    bool wasPressed = false;

    while (true) {
        // --- touch handling (act once per press) ---
        bool pressed = Brain.Screen.pressing();
        if (pressed) {
            int tx = Brain.Screen.xPosition();
            int ty = Brain.Screen.yPosition();
            if (!wasPressed) handleTouch(tx, ty);
            else if (currentPage == MOTOR_DIAGNOSTIC_PAGE) updateTouchControl(tx, ty);
        } else if (wasPressed) {
            clearTouchControl(); // joystick and speed slider are dead-man controls
        }
        wasPressed = pressed;

        // --- draw frame (double-buffered, so no flicker) ---
        takeSnapshot();
        Brain.Screen.clearScreen(color::black);

        if (currentPage == MOTOR_DIAGNOSTIC_PAGE && diagnosticMotor >= 0) {
            drawMotorPage(diagnosticMotor);
        } else if (currentPage == PORT_DIAGNOSTIC_PAGE &&
                   diagnosticPort >= MIN_PORT && diagnosticPort <= MAX_PORT) {
            drawPortDiagnosticPage(diagnosticPort);
        } else {
            currentPage = HOME_PAGE;
            drawSmartPortGrid();
        }

        Brain.Screen.render();
        this_thread::sleep_for(50);
    }
    return 0;
}

int main() {
    for (int i = 0; i < NUM_MOTORS; i++) rebuildMotor(i);
    for (int p = MIN_PORT; p <= MAX_PORT; p++) probes[p] = new device(p - 1);

    thread screenThread(screenTask);

    while (true) {
        int physicalFwd  = Controller1.Axis3.position();
        int physicalTurn = Controller1.Axis1.position();
        int fwdAxis  = touchStickLeft ? touchFwd : physicalFwd;
        int turnAxis = touchStickRight ? touchTurn : physicalTurn;

        double turn = curve(turnAxis, TURN_CURVE) * TURN_SCALE;

        double leftPower  = fwdAxis + turn;
        double rightPower = fwdAxis - turn;

        if (leftPower > 100) leftPower = 100;
        if (leftPower < -100) leftPower = -100;
        if (rightPower > 100) rightPower = 100;
        if (rightPower < -100) rightPower = -100;

        // A1/A2 claw pair: right triggers drive the motors in opposite directions.
        // R1 = A1 forward / A2 reverse; R2 = A1 reverse / A2 forward.
        bool l1 = Controller1.ButtonL1.pressing() || touchL1;
        bool l2 = Controller1.ButtonL2.pressing() || touchL2;
        bool r1 = Controller1.ButtonR1.pressing() || touchR1;
        bool r2 = Controller1.ButtonR2.pressing() || touchR2;
        double clawPower = 0;
        if (r1)      clawPower = 100;
        else if (r2) clawPower = -100;

        // B1 uses the D-pad (right forward, down reverse); right wins if both.
        bool b1Forward = Controller1.ButtonRight.pressing() || touchRight;
        bool b1Reverse = Controller1.ButtonDown.pressing() || touchDown;
        double b1Power = 0;
        if (b1Forward)      b1Power = 100;
        else if (b1Reverse) b1Power = -100;

        // L1/L2 run the B2 intake in opposite directions; L1 wins if both.
        double b2Power = 0;
        if (l1)      b2Power = 100;
        else if (l2) b2Power = -100;

        // Share the live input with the screen (CONTROL panel on the motor pages)
        inputFwd  = fwdAxis;
        inputTurn = turnAxis;
        inputL1   = l1;
        inputL2   = l2;
        inputR1   = r1;
        inputR2   = r2;
        motorCommand[0] = motorCommand[1] = (int)round(leftPower);
        motorCommand[2] = motorCommand[3] = (int)round(rightPower);
        motorCommand[AUX_A] = (int)clawPower;
        motorCommand[AUX_B] = (int)-clawPower;
        motorCommand[B1_MOTOR] = (int)b1Power;
        motorCommand[B2_MOTOR] = (int)b2Power;

        // Lock so a port change on the screen can't swap a motor out mid-update.
        motorLock.lock();
        double drivePower[4] = { leftPower, leftPower, rightPower, rightPower };
        for (int i = 0; i < 4; i++) {
            double power = drivePower[i];
            if (touchMotorActive && currentPage == MOTOR_DIAGNOSTIC_PAGE &&
                diagnosticMotor == i) {
                power = touchMotorPower;
                motorCommand[i] = (int)power;
            }
            if (power == 0) motors[i]->stop();
            else motors[i]->spin(forward, power, percent);
        }

        for (int i = AUX_A; i < NUM_MOTORS; i++) {
            double power = 0;
            if (i == AUX_A) power = clawPower;
            else if (i == AUX_B) power = -clawPower;
            else if (i == B1_MOTOR) power = b1Power;
            else if (i == B2_MOTOR) power = b2Power;
            // The on-screen speed slider temporarily overrides the selected motor.
            if (touchMotorActive && currentPage == MOTOR_DIAGNOSTIC_PAGE &&
                diagnosticMotor == i) {
                power = touchMotorPower;
                motorCommand[i] = (int)power;
            }
            if (power == 0) motors[i]->stop();
            else motors[i]->spin(forward, power, percent);
        }
        motorLock.unlock();

        wait(20, msec);
    }
}
