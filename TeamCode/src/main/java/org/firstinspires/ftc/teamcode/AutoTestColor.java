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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.faltech.FaltechRobot;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
@Autonomous(name = "ReportBeaconColorRt", group = "7079")
public class AutoTestColor extends LinearOpMode {

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
        //FTrobot.driveTrain.GoColor ("white", 0.5, 120);
        //sleep(5000);
        ElapsedTime timer = new ElapsedTime();
        int timeout = 30;
        robot.beaconSensorLt.enableLed(true);
        robot.beaconSensorRt.enableLed(true);
        while (timer.time() < timeout) {
            int first_red_level = robot.beaconSensorRt.red();
            int first_blue_level = robot.beaconSensorRt.blue();
            String BeaconColor1 = FTrobot.driveTrain.GetBeaconColorRt();
//            telemetry.addData("Beacon Color", BeaconColor1);
//            telemetry.addData("Red level:  ", first_red_level + " Blue: " + first_blue_level);
//            telemetry.update();
        }
/*        String BeaconColor = FTrobot.driveTrain.GetBeaconColorRt();
        if (
                BeaconColor.equals("red")
                ) {}
        else {
            FTrobot.driveTrain.GoInches(6, .5, 15);
        }
*/

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
