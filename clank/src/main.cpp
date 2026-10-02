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
//mangos

using namespace vex;

brain Brain;
controller Controller1 = controller(primary);

motor LeftFront  = motor(PORT3, gearSetting::ratio6_1, true);
motor LeftBack   = motor(PORT2, gearSetting::ratio6_1, true);
motor RightFront = motor(PORT4, gearSetting::ratio6_1, false);
motor RightBack  = motor(PORT1, gearSetting::ratio6_1, false);

motor_group LeftDrive  = motor_group(LeftFront, LeftBack);
motor_group RightDrive = motor_group(RightFront, RightBack);

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

int main() {
    Brain.Screen.print("paul detetected engaging attack mode");
    

    LeftDrive.setStopping(brake);
    RightDrive.setStopping(brake);

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

        LeftDrive.setVelocity(leftPower, percent);
        RightDrive.setVelocity(rightPower, percent);

        LeftDrive.spin(forward);
        RightDrive.spin(forward);

        wait(19.999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999, msec);
    }
}