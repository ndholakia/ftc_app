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
 @Autonomous(name = "AutoBasicStraight", group = "7079")

public class AutoBasicStraight extends LinearOpMode {

    private FaltechRobot FTrobot;
    PurpleBotHardware robot = new PurpleBotHardware();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        FTrobot = new FaltechRobot(this, robot);

        // wait for the start button to be pressed
        waitForStart();
/* This is a Autonomous for blue alliance,
It goes from the corner to drop off the climbers
into the shelter. Then it ends in the Floor Goal.
 */
        //Auto Start
        FTrobot.driveTrain.GoInches(20, .5, 6);
        FTrobot.driveTrain.GoInches(20, -.5, 6);


 //       robot.driveTrain.GoInches(60, .5, 15);
 //       robot.driveTrain.PivotTurn(-45, .5, 3);
 ////       robot.driveTrain.GoInches(6, -.5, 5);
       /* robot.arms.elevator.UpDegrees(30);
        robot.arms.hopper.goRight(4000);
        robot.arms.hopper.goLeft(4000);
        robot.arms.elevator.UpDegrees(0);
        robot.driveTrain.GoInches(34, -.5, 5);
*/

    }
}
