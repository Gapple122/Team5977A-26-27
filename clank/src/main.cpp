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
/*                                                                            */
/*  HOME = Smart Port grid. Tap a tile to inspect that port.                  */
/*  SWITCH PORTS enables motor port reassignment by tapping a motor and then  */
/*  a destination port. Motor assignments swap when the destination is busy. */
/*  3WIRE = A-H list.                                                        */
/*----------------------------------------------------------------------------*/

const int NUM_3WIRE = 8;
const char* threeWireNames[NUM_3WIRE] = {
    "A", "B", "C", "D", "E", "F", "G", "H"
};

/*
 * 3-Wire devices cannot be generically identified from the Brain like Smart
 * Port devices. These names are therefore labels for configured connections.
 * Change them to match the devices used by your robot.
 *
 * Example:
 *   "Limit Switch"
 *   "Potentiometer"
 *   "Encoder (A+B)"
 */
const char* threeWireDevices[NUM_3WIRE] = {
    "Not configured", "Not configured", "Not configured", "Not configured",
    "Not configured", "Not configured", "Not configured", "Not configured"
};

const int NUM_PAGES = NUM_MOTORS + 3;   // HOME + 3WIRE + HELP + each motor
const char* buttonLabels[NUM_PAGES] = {
    "HOME", "3WIRE", "MOTORS", "LF", "LB", "RF", "RB"
};

const int BTN_X = 0;
const int BTN_W = 100;
const int BTN_H = 240 / NUM_PAGES;
const int PANEL_X = 110;

/* Smart-port grid: 7 columns x 3 rows = 21 tiles */
const int TILE_COLS = 7;
const int TILE_ROWS = 3;
const int TILE_W = 50;
const int TILE_H = 52;
const int TILE_X0 = 112;
const int TILE_Y0 = 48;

/* Smart-port detail panel */
const int DETAIL_X = 112;
const int DETAIL_Y = 208;
const int SWITCH_X = 365;
const int SWITCH_Y = 8;
const int SWITCH_W = 105;
const int SWITCH_H = 30;

int currentPage = 0;
int selectedMotor = -1;       // Motor selected while SWITCH PORTS is enabled
int selectedPort = -1;        // Port whose details are displayed
bool switchPortsMode = false;

/* Snapshot of motor readings, so drawing never holds the lock the drive loop needs. */
struct MotorData {
    bool installed;
    double temp, rpm, amps, volts, torque, watts, eff, pos;
};
MotorData snap[NUM_MOTORS];

/* Generic Smart-Port information. */
struct PortData {
    bool installed;
    int type;
};
PortData portSnap[MAX_PORT + 1];

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

        // Keep the old arrays updated too.
        portInstalled[p] = portSnap[p].installed;
        portType[p] = portSnap[p].type;
    }
}

/* Generic type names for the most useful/common Smart-Port cases. */
const char* deviceTypeName(int type) {
    switch (type) {
        case 0: return "None";
        case DEVICE_TYPE_MOTOR: return "Motor";
        default: return "Device";
    }
}

/* Green = cool, yellow = warm, red = hot. */
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
    Brain.Screen.printAt(x + 8, y + h / 2 + 7, true, "%s", label);
}

void drawPageButtons() {
    Brain.Screen.setFont(mono15);

    for (int i = 0; i < NUM_PAGES; i++) {
        drawButton(BTN_X, i * BTN_H, BTN_W, BTN_H, buttonLabels[i],
                   i == currentPage ? color::blue : color(50, 50, 50));
    }
}

void drawSmartPortGrid() {
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.setFont(mono20);
    Brain.Screen.printAt(PANEL_X + 10, 25, true, "SMART PORTS");

    Brain.Screen.setFont(mono15);
    drawButton(SWITCH_X, SWITCH_Y, SWITCH_W, SWITCH_H,
               switchPortsMode ? "MOVE: ON" : "SWITCH PORTS",
               switchPortsMode ? color(0, 110, 0) : color(60, 60, 60));

    if (switchPortsMode) {
        Brain.Screen.setPenColor(color::yellow);
        Brain.Screen.printAt(PANEL_X + 10, 42, true, "Tap motor, then destination");
    } else if (selectedPort >= MIN_PORT && selectedPort <= MAX_PORT) {
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10, 42, true, "Tap a port for details");
    } else {
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10, 42, true, "Tap any port");
    }

    for (int p = MIN_PORT; p <= MAX_PORT; p++) {
        int col = (p - 1) % TILE_COLS;
        int row = (p - 1) / TILE_COLS;
        int x = TILE_X0 + col * TILE_W;
        int y = TILE_Y0 + row * TILE_H;

        int m = motorAtPort(p);

        color fill = color(40, 40, 40);
        const char* label = "";

        if (m >= 0) {
            fill = snap[m].installed ? color(0, 110, 0) : color(140, 0, 0);
            label = buttonLabels[m + 3];
        } else if (portSnap[p].installed) {
            fill = color(0, 70, 140);
            label = deviceTypeName(portSnap[p].type);
        }

        bool selected = (selectedPort == p);
        bool selectedMoveMotor = (selectedMotor >= 0 && m == selectedMotor);

        Brain.Screen.setPenWidth((selected || selectedMoveMotor) ? 3 : 1);
        Brain.Screen.setPenColor(
            (selected || selectedMoveMotor) ? color::yellow : color(90, 90, 90)
        );
        Brain.Screen.setFillColor(fill);
        Brain.Screen.drawRectangle(x, y, TILE_W - 2, TILE_H - 2);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.setFont(mono20);
        Brain.Screen.printAt(x + 5, y + 22, true, "%2d", p);

        Brain.Screen.setFont(mono15);
        if (label[0] != '\0') {
            Brain.Screen.printAt(x + 4, y + 43, true, "%s", label);
        }
    }

    Brain.Screen.setPenWidth(1);
    Brain.Screen.setFont(mono15);

    /* Detail area below the grid. */
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::white);

    if (selectedMotor >= 0 && switchPortsMode) {
        Brain.Screen.printAt(DETAIL_X, DETAIL_Y, true,
            "Moving %s from P%d",
            motorNames[selectedMotor], motorPorts[selectedMotor]);
        Brain.Screen.printAt(DETAIL_X, DETAIL_Y + 20, true, "Tap destination or MOVE to cancel");
        return;
    }

    if (selectedPort < MIN_PORT || selectedPort > MAX_PORT) {
        Brain.Screen.printAt(DETAIL_X, DETAIL_Y, true, "No port selected");
        return;
    }

    int p = selectedPort;
    int m = motorAtPort(p);

    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.printAt(DETAIL_X, DETAIL_Y, true, "PORT %d", p);

    Brain.Screen.setPenColor(color::white);

    if (m >= 0) {
        Brain.Screen.printAt(DETAIL_X, DETAIL_Y + 20, true, "Assigned: %s", motorNames[m]);

        if (!snap[m].installed) {
            Brain.Screen.setPenColor(color::red);
            Brain.Screen.printAt(DETAIL_X, DETAIL_Y + 40, true, "Motor not detected");
        } else {
            Brain.Screen.setPenColor(tempColor(snap[m].temp));
            Brain.Screen.printAt(DETAIL_X, DETAIL_Y + 40, true, "Motor: %.0fC %.0f rpm", snap[m].temp, snap[m].rpm);
        }
    } else if (portSnap[p].installed) {
        Brain.Screen.printAt(DETAIL_X, DETAIL_Y + 20, true, "Device: %s", deviceTypeName(portSnap[p].type));
        Brain.Screen.printAt(DETAIL_X, DETAIL_Y + 40, true, "Type ID: %d", portSnap[p].type);
    } else {
        Brain.Screen.printAt(DETAIL_X, DETAIL_Y + 20, true, "No device detected");
    }
}

void drawThreeWirePage() {
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.setFont(mono20);
    Brain.Screen.printAt(PANEL_X + 10, 24, true, "3-WIRE PORTS");

    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.printAt(PANEL_X + 10, 40, true, "Select a port for its configured connection");

    const int colW = 168;
    const int rowH = 34;
    const int listY = 48;

    for (int i = 0; i < NUM_3WIRE; i++) {
        int col = i / 4;
        int row = i % 4;
        int x = PANEL_X + 8 + col * (colW + 8);
        int y = listY + row * rowH;
        bool selected = (selectedPort == -(i + 1));

        Brain.Screen.setPenColor(selected ? color::yellow : color(90, 90, 90));
        Brain.Screen.setFillColor(selected ? color(70, 70, 20) : color(35, 35, 35));
        Brain.Screen.drawRectangle(x, y, colW, rowH - 3);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.setFont(mono20);
        Brain.Screen.printAt(x + 8, y + 23, true, "%c", 'A' + i);

        Brain.Screen.setFont(mono15);
        Brain.Screen.printAt(x + 38, y + 15, true, "%s", threeWireDevices[i]);
    }

    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color::cyan);

    if (selectedPort < 0 && selectedPort >= -NUM_3WIRE) {
        int i = -selectedPort - 1;
        Brain.Screen.printAt(PANEL_X + 10, 198, true, "Port %c", 'A' + i);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10, 218, true, "Connection: %s", threeWireDevices[i]);
    } else {
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10, 198, true, "No 3-wire port selected");
    }

    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color(150, 150, 150));
    Brain.Screen.printAt(PANEL_X + 10, 236, true, "3-wire hardware is not generically auto-detectable");
}

void drawMotorSummaryPage() {
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setPenColor(color::cyan);
    Brain.Screen.setFont(mono20);
    Brain.Screen.printAt(PANEL_X + 10, 24, true, "MOTOR GRID");

    const int cols = 2;
    const int w = 125;
    const int h = 78;
    const int x0 = PANEL_X + 8;
    const int y0 = 48;

    for (int i = 0; i < NUM_MOTORS; i++) {
        int col = i % cols;
        int row = i / cols;
        int x = x0 + col * (w + 8);
        int y = y0 + row * (h + 8);

        bool selected = (selectedMotor == i && switchPortsMode);

        Brain.Screen.setPenWidth(selected ? 3 : 1);
        Brain.Screen.setPenColor(selected ? color::yellow : color(90, 90, 90));
        Brain.Screen.setFillColor(snap[i].installed ? color(0, 90, 0) : color(110, 0, 0));
        Brain.Screen.drawRectangle(x, y, w, h);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.setFont(mono15);
        Brain.Screen.printAt(x + 8, y + 20, true, "%s", motorNames[i]);
        Brain.Screen.printAt(x + 8, y + 40, true, "Port: %d", motorPorts[i]);

        if (snap[i].installed) {
            Brain.Screen.printAt(x + 8, y + 60, true, "%.0fC %.0f rpm",
                                 snap[i].temp, snap[i].rpm);
        } else {
            Brain.Screen.setPenColor(color::red);
            Brain.Screen.printAt(x + 8, y + 60, true, "NOT DETECTED");
        }
    }

    Brain.Screen.setPenWidth(1);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.printAt(PANEL_X + 10, 224, true, "Use HOME > SWITCH PORTS to move assignments");
}

void drawMotorPage(int idx) {
    const MotorData& d = snap[idx];

    Brain.Screen.setFillColor(color::black);
    Brain.Screen.setFont(mono20);

    Brain.Screen.setPenColor(portInUse(idx) ? color::red : color::cyan);
    Brain.Screen.printAt(PANEL_X + 10, 24, true,
        "%s  (port %d)", motorNames[idx], motorPorts[idx]);

    if (!d.installed) {
        Brain.Screen.setPenColor(color::red);
        Brain.Screen.printAt(PANEL_X + 10, 62, true, "NOT DETECTED");
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10, 90, true, "Use HOME to switch ports");
    } else {
        Brain.Screen.setPenColor(tempColor(d.temp));
        Brain.Screen.printAt(PANEL_X + 10, 54, true, "Temp:       %3.0f C", d.temp);

        Brain.Screen.setPenColor(color::white);
        Brain.Screen.printAt(PANEL_X + 10, 78, true, "Velocity:   %5.0f rpm", d.rpm);
        Brain.Screen.printAt(PANEL_X + 10, 102, true, "Current:    %5.2f A", d.amps);
        Brain.Screen.printAt(PANEL_X + 10, 126, true, "Voltage:    %5.2f V", d.volts);
        Brain.Screen.printAt(PANEL_X + 10, 150, true, "Torque:     %5.2f Nm", d.torque);
        Brain.Screen.printAt(PANEL_X + 10, 174, true, "Power:      %5.1f W", d.watts);
        Brain.Screen.printAt(PANEL_X + 10, 198, true, "Efficiency: %5.0f %%", d.eff);
        Brain.Screen.printAt(PANEL_X + 10, 222, true, "Position:   %5.0f deg", d.pos);
    }
}

void handleSmartGridTouch(int x, int y) {
    if (inRect(x, y, SWITCH_X, SWITCH_Y, SWITCH_W, SWITCH_H)) {
        switchPortsMode = !switchPortsMode;
        selectedMotor = -1;
        selectedPort = -1;
        return;
    }

    if (!inRect(x, y, TILE_X0, TILE_Y0,
                TILE_COLS * TILE_W, TILE_ROWS * TILE_H)) {
        return;
    }

    int col = (x - TILE_X0) / TILE_W;
    int row = (y - TILE_Y0) / TILE_H;
    int port = row * TILE_COLS + col + 1;

    if (port < MIN_PORT || port > MAX_PORT) return;

    int m = motorAtPort(port);

    if (switchPortsMode) {
        if (selectedMotor < 0) {
            if (m >= 0) {
                selectedMotor = m;
                selectedPort = port;
            }
        } else if (port == motorPorts[selectedMotor]) {
            selectedMotor = -1;
        } else {
            moveMotorToPort(selectedMotor, port);
            selectedMotor = -1;
            selectedPort = port;
        }
    } else {
        selectedPort = port;
    }
}

void handleThreeWireTouch(int x, int y) {
    const int colW = 168;
    const int rowH = 34;
    const int listY = 48;

    if (!inRect(x, y, PANEL_X + 8, listY,
                2 * colW + 8, 4 * rowH)) {
        return;
    }

    int col = (x - (PANEL_X + 8)) / (colW + 8);
    int row = (y - listY) / rowH;

    if (col < 0 || col > 1 || row < 0 || row > 3) return;

    int index = col * 4 + row;
    if (index >= 0 && index < NUM_3WIRE) {
        selectedPort = -(index + 1);
    }
}

void handleTouch(int x, int y) {
    if (inRect(x, y, BTN_X, 0, BTN_W, 240)) {
        int page = y / BTN_H;

        if (page >= 0 && page < NUM_PAGES) {
            currentPage = page;
            selectedMotor = -1;
            selectedPort = -1;
            switchPortsMode = false;
        }
        return;
    }

    if (currentPage == 0) {
        handleSmartGridTouch(x, y);
    } else if (currentPage == 1) {
        handleThreeWireTouch(x, y);
    } else if (currentPage == 2) {
        // Motor grid: selecting a motor only changes the selected card.
        const int cols = 2;
        const int w = 125;
        const int h = 78;
        const int x0 = PANEL_X + 8;
        const int y0 = 48;

        for (int i = 0; i < NUM_MOTORS; i++) {
            int col = i % cols;
            int row = i / cols;
            int bx = x0 + col * (w + 8);
            int by = y0 + row * (h + 8);

            if (inRect(x, y, bx, by, w, h)) {
                selectedMotor = i;
                return;
            }
        }
    } else if (currentPage >= 3) {
        int idx = currentPage - 3;
        if (idx >= 0 && idx < NUM_MOTORS) {
            // Motor pages no longer have independent port +/- controls.
            // Port movement is handled from HOME > SWITCH PORTS.
        }
    }
}

/* Runs on its own thread so drawing never slows the drive loop. */
int screenTask() {
    bool wasPressed = false;

    while (true) {
        bool pressed = Brain.Screen.pressing();

        if (pressed && !wasPressed) {
            handleTouch(Brain.Screen.xPosition(), Brain.Screen.yPosition());
        }
        wasPressed = pressed;

        takeSnapshot();

        Brain.Screen.clearScreen(color::black);
        drawPageButtons();

        if (currentPage == 0) {
            drawSmartPortGrid();
        } else if (currentPage == 1) {
            drawThreeWirePage();
        } else if (currentPage == 2) {
            drawMotorSummaryPage();
        } else {
            drawMotorPage(currentPage - 3);
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