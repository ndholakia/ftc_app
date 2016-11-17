/*

FALTECH 7079
FOUR POINTS MIDDLE SCHOOL

AUTONOMOUS CLASS NAMING NOMENCLATURE:
    OpMode Type: {Auto|Telop}
    Alliance color: {R|B} // Red or Blue
    Starting Position: {CS|M|MS} // Corner Side or Middle or Mountain Side
    Scoring strategy: {guys|none|beacon}
    End location: {PZ|SZ} // Parking Zone or Safety Zone

DESCRIPTION:
    Autonomous program for Blue alliance, starting in Corner Side, scoring the climbers in the box,
    and finishing in the Parking Zone.
TARGET SCORE:
45

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.faltech.FaltechRobot;

/**
 * A simple example of a linear
 */
 @Autonomous(name = "Red_Far_Beacon", group = "7079")

public class AutoRedFarBeacon extends LinearOpMode {

    private FaltechRobot FTrobot;
    PurpleBotHardware robot = new PurpleBotHardware();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        FTrobot = new FaltechRobot(this, robot);
        FTrobot.driveTrain.SuperAuto("red", false, 1, false);
    }
}
