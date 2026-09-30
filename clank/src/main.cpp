/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       prane                                                     */
/*    Created:      9/27/2026, 4:08:57 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
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

int main() {
    Brain.Screen.print("Split Arcade Drive Ready");

    LeftDrive.setStopping(coast);
    RightDrive.setStopping(coast);

    while (true) {
        int fwdAxis = Controller1.Axis3.position(); // left stick, vertical
        int turnAxis = Controller1.Axis1.position(); // right stick, horizontal

        int leftPower  = fwdAxis + turnAxis;
        int rightPower = fwdAxis - turnAxis;

        if (leftPower > 100) leftPower = 100;
        if (leftPower < -100) leftPower = -100;
        if (rightPower > 100) rightPower = 100;
        if (rightPower < -100) rightPower = -100;

        // Set velocity first (magnitude), then spin in the forward direction.
        // Negative velocity values will make it spin backward automatically.
        LeftDrive.setVelocity(leftPower, percent);
        RightDrive.setVelocity(rightPower, percent);

        LeftDrive.spin(forward);
        RightDrive.spin(forward);

        wait(20, msec);
    }
}