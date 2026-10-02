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
/*  uses. Every port change (HOME grid or PORT +/-) updates it and rebuilds   */
/*  the motor on the new port.                                                */
/*----------------------------------------------------------------------------*/

const int NUM_MOTORS = 4;
const int MIN_PORT = 1;
const int MAX_PORT = 21;

const char* motorNames[NUM_MOTORS] = { "L Front", "L Back", "R Front", "R Back" };

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

// Sets motor i to a specific port. Does not check for duplicates.
void setMotorPort(int i, int port) {
    if (port > MAX_PORT) port = MIN_PORT;
    if (port < MIN_PORT) port = MAX_PORT;
    if (port == motorPorts[i]) return;
    motorLock.lock();
    motorPorts[i] = port;
    rebuildMotor(i);
    motorLock.unlock();
}

// PORT + / PORT - buttons
void changePort(int i, int delta) {
    setMotorPort(i, motorPorts[i] + delta);
}

// HOME grid: move motor i to a port. If another motor is already there, the two swap.
void moveMotorToPort(int i, int port) {
    int other = motorAtPort(port);
    if (other >= 0 && other != i) {
        motorLock.lock();
        motorPorts[other] = motorPorts[i];
        motorPorts[i] = port;
        rebuildMotor(other);
        rebuildMotor(i);
        motorLock.unlock();
    } else {
        setMotorPort(i, port);
    }
}

void toggleReverse(int i) {
    motorLock.lock();
    motorReversed[i] = !motorReversed[i];
    rebuildMotor(i);
    motorLock.unlock();
}

bool portInUse(int i) {
    for (int j = 0; j < NUM_MOTORS; j++) {
        if (j != i && motorPorts[j] == motorPorts[i]) return true;
    }
    return false;
}

/*----------------------------------------------------------------------------*/
/*  Port probes: one generic device handle per port, used by the HOME screen  */
/*  to show what is physically plugged in (even if the code doesn't use it).  */
/*----------------------------------------------------------------------------*/

const int DEVICE_TYPE_MOTOR = 2;   // V5 device type id for a motor

device* probes[MAX_PORT + 1] = { nullptr };   // indexed 1..21
bool portInstalled[MAX_PORT + 1];
int  portType[MAX_PORT + 1];

/*----------------------------------------------------------------------------*/
/*  Brain screen GUI                                                          */
/*  Left column = page buttons. Right panel = data.                           */
/*  Page 0 = HOME (all 21 ports). Page 1 = ALL motors summary.                */
/*  Pages 2-5 = one motor, with PORT +/- and REV buttons on the right edge.   */
/*----------------------------------------------------------------------------*/

const int NUM_PAGES = NUM_MOTORS + 2;   // HOME + ALL + each motor
const char* buttonLabels[NUM_PAGES] = { "HOME", "ALL", "LF", "LB", "RF", "RB" };

const int BTN_X = 0;
const int BTN_W = 100;
const int BTN_H = 240 / NUM_PAGES;   // 40 px each
const int PANEL_X = 110;

// Port/reverse buttons (shown on motor pages only)
const int PBTN_X = 365;
const int PBTN_W = 110;
const int PBTN_H = 48;
const int PBTN_PLUS_Y  = 40;
const int PBTN_MINUS_Y = 98;
const int PBTN_REV_Y   = 156;

// HOME port grid: 7 columns x 3 rows = 21 tiles
const int TILE_COLS = 7;
const int TILE_ROWS = 3;
const int TILE_W  = 52;
const int TILE_H  = 58;
const int TILE_X0 = 112;
const int TILE_Y0 = 36;

int currentPage = 0;
int selectedMotor = -1;   // motor picked on the HOME screen, waiting for a new port

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
        portInstalled[p] = probes[p]->installed();
        portType[p] = portInstalled[p] ? (int)probes[p]->type() : 0;
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

void drawButton(int x, int y, int w, int h, const char* label, color fill) {
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setFillColor(fill);
    Brain.Screen.drawRectangle(x, y, w, h);
    Brain.Screen.printAt(x + 10, y + h / 2 + 7, true, "%s", label);
}

void drawPageButtons() {
    Brain.Screen.setFont(mono20);
    for (int i = 0; i < NUM_PAGES; i++) {
        drawButton(BTN_X, i * BTN_H, BTN_W, BTN_H, buttonLabels[i],
                   i == currentPage ? color::blue : color(50, 50, 50));
    }
}

void drawHomePage() {
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.setFont(mono20);
    Brain.Screen.printAt(PANEL_X + 10, 24, "PORTS");

    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color::white);
    if (selectedMotor >= 0) {
        Brain.Screen.printAt(PANEL_X + 90, 22, true, "Move %s: tap new port", motorNames[selectedMotor]);
    } else {
        Brain.Screen.printAt(PANEL_X + 90, 22, "Tap a motor to move it");
    }

    for (int p = MIN_PORT; p <= MAX_PORT; p++) {
        int col = (p - 1) % TILE_COLS;
        int row = (p - 1) / TILE_COLS;
        int x = TILE_X0 + col * TILE_W;
        int y = TILE_Y0 + row * TILE_H;

        int m = motorAtPort(p);
        color fill = color(40, 40, 40);      // empty port
        const char* label = "";

        if (m >= 0) {
            fill  = snap[m].installed ? color(0, 110, 0) : color(140, 0, 0);
            label = buttonLabels[m + 2];     // LF / LB / RF / RB
        } else if (portInstalled[p]) {
            fill  = color(0, 70, 140);       // something plugged in that the code doesn't use
            label = (portType[p] == DEVICE_TYPE_MOTOR) ? "mtr" : "dev";
        }

        bool selected = (m >= 0 && m == selectedMotor);
        Brain.Screen.setPenWidth(selected ? 3 : 1);
        Brain.Screen.setPenColor(selected ? color::yellow : color(90, 90, 90));
        Brain.Screen.setFillColor(fill);
        Brain.Screen.drawRectangle(x, y, TILE_W - 2, TILE_H - 2);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.setFont(mono20);
        Brain.Screen.printAt(x + 6, y + 24, "%2d", p);
        if (label[0] != '\0') {
            Brain.Screen.setFont(mono15);
            Brain.Screen.printAt(x + 6, y + 46, true, "%s", label);
        }
    }
    Brain.Screen.setPenWidth(1);

    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setFont(mono15);
    Brain.Screen.printAt(PANEL_X + 10, 232, "green=ok red=missing blue=unassigned");
}

void drawSummaryPage() {
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.setFont(mono20);
    Brain.Screen.printAt(PANEL_X + 10, 24, "ALL MOTORS");

    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setFont(mono15);
    Brain.Screen.printAt(PANEL_X + 10,  54, "Motor");
    Brain.Screen.printAt(PANEL_X + 90,  54, "Port");
    Brain.Screen.printAt(PANEL_X + 140, 54, "Temp");
    Brain.Screen.printAt(PANEL_X + 195, 54, "RPM");
    Brain.Screen.printAt(PANEL_X + 250, 54, "Amps");

    for (int i = 0; i < NUM_MOTORS; i++) {
        int y = 84 + i * 30;

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10, y, true, "%-8s", motorNames[i]);

        Brain.Screen.setPenColor(portInUse(i) ? color::red : color::white);
        Brain.Screen.printAt(PANEL_X + 90, y, "%2d", motorPorts[i]);

        if (!snap[i].installed) {
            Brain.Screen.setPenColor(color::red);
            Brain.Screen.printAt(PANEL_X + 140, y, "not detected");
            continue;
        }

        Brain.Screen.setPenColor(tempColor(snap[i].temp));
        Brain.Screen.printAt(PANEL_X + 140, y, "%3.0fC", snap[i].temp);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 195, y, "%4.0f", snap[i].rpm);
        Brain.Screen.printAt(PANEL_X + 250, y, "%4.2f", snap[i].amps);
    }

    Brain.Screen.setPenColor(color::orange);
    Brain.Screen.printAt(PANEL_X + 10, 220, "paul detetected engaging attack mode");
}

void drawMotorPage(int idx) {
    const MotorData& d = snap[idx];

    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono20);

    // Title turns red if two motors are assigned to the same port
    Brain.Screen.setPenColor(portInUse(idx) ? color::red : color::cyan);
    Brain.Screen.printAt(PANEL_X + 10, 24, true, "%s  (port %d)", motorNames[idx], motorPorts[idx]);

    if (!d.installed) {
        Brain.Screen.setPenColor(color::red);
        Brain.Screen.printAt(PANEL_X + 10, 62, "NOT DETECTED");
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10, 90, "Try PORT + / -");
    } else {
        Brain.Screen.setPenColor(tempColor(d.temp));
        Brain.Screen.printAt(PANEL_X + 10, 54, "Temp:       %3.0f C", d.temp);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10,  78, "Velocity:   %5.0f rpm", d.rpm);
        Brain.Screen.printAt(PANEL_X + 10, 102, "Current:    %5.2f A",   d.amps);
        Brain.Screen.printAt(PANEL_X + 10, 126, "Voltage:    %5.2f V",   d.volts);
        Brain.Screen.printAt(PANEL_X + 10, 150, "Torque:     %5.2f Nm",  d.torque);
        Brain.Screen.printAt(PANEL_X + 10, 174, "Power:      %5.1f W",   d.watts);
        Brain.Screen.printAt(PANEL_X + 10, 198, "Efficiency: %5.0f %%",  d.eff);
        Brain.Screen.printAt(PANEL_X + 10, 222, "Position:   %5.0f deg", d.pos);
    }

    // Port / reverse controls
    Brain.Screen.setFont(mono20);
    drawButton(PBTN_X, PBTN_PLUS_Y,  PBTN_W, PBTN_H, "PORT +", color(0, 110, 0));
    drawButton(PBTN_X, PBTN_MINUS_Y, PBTN_W, PBTN_H, "PORT -", color(130, 0, 0));
    drawButton(PBTN_X, PBTN_REV_Y,   PBTN_W, PBTN_H,
               motorReversed[idx] ? "REV: YES" : "REV: NO", color(70, 70, 70));
    Brain.Screen.setFillColor(color::black);
}

void handleHomeTouch(int x, int y) {
    if (!inRect(x, y, TILE_X0, TILE_Y0, TILE_COLS * TILE_W, TILE_ROWS * TILE_H)) return;

    int col = (x - TILE_X0) / TILE_W;
    int row = (y - TILE_Y0) / TILE_H;
    int port = row * TILE_COLS + col + 1;
    if (port < MIN_PORT || port > MAX_PORT) return;

    if (selectedMotor < 0) {
        // Nothing selected yet: tapping a motor's port selects it
        selectedMotor = motorAtPort(port);   // -1 if the port has no motor assigned
    } else if (port == motorPorts[selectedMotor]) {
        selectedMotor = -1;                  // tapped it again: cancel
    } else {
        moveMotorToPort(selectedMotor, port);
        selectedMotor = -1;
    }
}

void handleTouch(int x, int y) {
    // Page buttons
    if (inRect(x, y, BTN_X, 0, BTN_W, 240)) {
        int page = y / BTN_H;
        if (page >= 0 && page < NUM_PAGES) {
            currentPage = page;
            selectedMotor = -1;
        }
        return;
    }

    if (currentPage == 0) {
        handleHomeTouch(x, y);
    } else if (currentPage >= 2) {
        int idx = currentPage - 2;
        if      (inRect(x, y, PBTN_X, PBTN_PLUS_Y,  PBTN_W, PBTN_H)) changePort(idx, +1);
        else if (inRect(x, y, PBTN_X, PBTN_MINUS_Y, PBTN_W, PBTN_H)) changePort(idx, -1);
        else if (inRect(x, y, PBTN_X, PBTN_REV_Y,   PBTN_W, PBTN_H)) toggleReverse(idx);
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
        drawPageButtons();
        if      (currentPage == 0) drawHomePage();
        else if (currentPage == 1) drawSummaryPage();
        else                       drawMotorPage(currentPage - 2);
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