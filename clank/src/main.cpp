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
/*  Index 0,1 = left side.  Index 2,3 = right side.                           */
/*                                                                            */
/*  motorPorts[] is the single source of truth for which port each motor      */
/*  uses. Moving a motor on the screen updates it and rebuilds the motor on   */
/*  the new port, so the drive loop follows immediately.                      */
/*----------------------------------------------------------------------------*/

const int NUM_MOTORS = 4;
const int MIN_PORT = 1;
const int MAX_PORT = 21;

const char* motorNames[NUM_MOTORS] = { "L Front", "L Back", "R Front", "R Back" };
const char* motorShort[NUM_MOTORS] = { "LF", "LB", "RF", "RB" };

// Starting ports (1-21) and directions. Changed live from the Brain screen.
int  motorPorts[NUM_MOTORS]    = { 3, 2, 4, 1 };
bool motorReversed[NUM_MOTORS] = { true, true, false, false };

motor* motors[NUM_MOTORS] = { nullptr, nullptr, nullptr, nullptr };

// Protects the motors[] pointers: the drive loop and the screen thread both use them.
vex::mutex motorLock;

// (Re)creates motor i from its current port/reversed settings. Caller must hold motorLock
// (or be running before the screen thread starts).
void rebuildMotor(int i) {
    if (motors[i] != nullptr) {
        motors[i]->stop();
        delete motors[i];
    }
    // SDK port indexes are 0-based: PORT1 == 0
    motors[i] = new motor(motorPorts[i] - 1, gearSetting::ratio6_1, motorReversed[i]);
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

// SWITCH button
const int SWITCH_X = 376;
const int SWITCH_Y = 4;
const int SWITCH_W = 96;
const int SWITCH_H = 26;

// Smart-port grid: 7 columns x 3 rows = 21 tiles (y 52..220)
const int TILE_COLS = 7;
const int TILE_ROWS = 3;
const int TILE_W = 52;
const int TILE_H = 56;
const int TILE_X0 = 112;
const int TILE_Y0 = 52;

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

    // SWITCH button
    Brain.Screen.setFont(mono15);
    drawButton(SWITCH_X, SWITCH_Y, SWITCH_W, SWITCH_H,
               switchPortsMode ? "DONE" : "SWITCH",
               switchPortsMode ? color(0, 110, 0) : color(60, 60, 60), 9, 15);

    // Hint line (sits between the header and the grid)
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono15);
    if (switchPortsMode && selectedMotor >= 0) {
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
            fill = portSnap[p].installed ? color(0, 110, 0) : color(140, 0, 0);
            label = motorShort[m];               // LF / LB / RF / RB
        } else if (portSnap[p].installed) {
            fill = color(0, 70, 140);            // plugged in, not used by the code
            label = deviceShortName(portSnap[p].type);
        }

        bool held = (switchPortsMode && m >= 0 && m == selectedMotor);

        Brain.Screen.setPenWidth(held ? 3 : 1);
        Brain.Screen.setPenColor(held ? color::yellow : color(90, 90, 90));
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
    Brain.Screen.printAt(PANEL_X, 236, true, "green=ok red=missing blue=other");
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
    Brain.Screen.printAt(10, 24, true, "%s  (port %d)", motorNames[idx], motorPorts[idx]);

    if (!d.installed) {
        Brain.Screen.setPenColor(color::red);
        Brain.Screen.printAt(10, 62, true, "NOT DETECTED");
        Brain.Screen.setFont(mono15);
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(10, 90, true, "Check the cable and port.");
    } else {
        Brain.Screen.setPenColor(tempColor(d.temp));
        Brain.Screen.printAt(10, 54, true, "Temp:       %3.0f C", d.temp);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(10,  78, true, "Velocity:   %5.0f rpm", d.rpm);
        Brain.Screen.printAt(10, 102, true, "Current:    %5.2f A",   d.amps);
        Brain.Screen.printAt(10, 126, true, "Voltage:    %5.2f V",   d.volts);
        Brain.Screen.printAt(10, 150, true, "Torque:     %5.2f Nm",  d.torque);
        Brain.Screen.printAt(10, 174, true, "Power:      %5.1f W",   d.watts);
        Brain.Screen.printAt(10, 198, true, "Efficiency: %5.0f %%",  d.eff);
        Brain.Screen.printAt(10, 222, true, "Position:   %5.0f deg", d.pos);
    }

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
        switchPortsMode = !switchPortsMode;
        selectedMotor = -1;
        return;
    }

    if (!inRect(x, y, TILE_X0, TILE_Y0, TILE_COLS * TILE_W, TILE_ROWS * TILE_H)) return;

    int col = (x - TILE_X0) / TILE_W;
    int row = (y - TILE_Y0) / TILE_H;
    int port = row * TILE_COLS + col + 1;
    if (port < MIN_PORT || port > MAX_PORT) return;

    int m = motorAtPort(port);

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

void handleTouch(int x, int y) {
    if (currentPage == HOME_PAGE) {
        if (x < DIVIDER_X) handleThreeWireTouch(x, y);
        else               handleSmartGridTouch(x, y);
        return;
    }

    // Diagnostic screens: BACK returns to HOME
    if (inRect(x, y, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        currentPage = HOME_PAGE;
        diagnosticMotor = -1;
        diagnosticPort = -1;
        selectedMotor = -1;
        switchPortsMode = false;
    }
}

// Runs on its own thread so drawing never slows the drive loop.
int screenTask() {
    bool wasPressed = false;

    while (true) {
        // --- touch handling (act once per press) ---
        bool pressed = Brain.Screen.pressing();
        if (pressed && !wasPressed) {
            handleTouch(Brain.Screen.xPosition(), Brain.Screen.yPosition());
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
        int fwdAxis  = Controller1.Axis3.position(); // left stick, vertical
        int turnAxis = Controller1.Axis1.position(); // right stick, horizontal

        double turn = curve(turnAxis, TURN_CURVE) * TURN_SCALE;

        double leftPower  = fwdAxis + turn;
        double rightPower = fwdAxis - turn;

        if (leftPower > 100) leftPower = 100;
        if (leftPower < -100) leftPower = -100;
        if (rightPower > 100) rightPower = 100;
        if (rightPower < -100) rightPower = -100;

        // Lock so a port change on the screen can't swap a motor out mid-update.
        motorLock.lock();
        motors[0]->spin(forward, leftPower,  percent);   // left front
        motors[1]->spin(forward, leftPower,  percent);   // left back
        motors[2]->spin(forward, rightPower, percent);   // right front
        motors[3]->spin(forward, rightPower, percent);   // right back
        motorLock.unlock();

        wait(20, msec);
    }
}