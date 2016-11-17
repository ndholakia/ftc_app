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
 @Autonomous(name = "Red_Both_Beacons_Center", group = "7079")

public class AutoRedBothBeaconsCenter extends LinearOpMode {

    private FaltechRobot FTrobot;
    PurpleBotHardware robot = new PurpleBotHardware();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        FTrobot = new FaltechRobot(this, robot);

        // wait for the start button to be pressed
        //waitForStart();
/* This is a Autonomous for blue alliance,
 */
        FTrobot.driveTrain.SuperAuto("red", false, 2, true);
        /*
        //Auto Start
        robot.mtrLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrLeft.setMaxSpeed(2240);
        robot.mtrRight.setMaxSpeed(2240);
        FTrobot.driveTrain.srvMove(-1);
        FTrobot.driveTrain.GoInches(6, .6, 6);
        FTrobot.driveTrain.PivotTurn(-56, .18, false, 5);
        telemetry.addData("1.  Starting:  ", " long angle");
        telemetry.update();
        sleep(1000);
        FTrobot.driveTrain.GoInches(55, 1, 5);
        FTrobot.driveTrain.GoImpact(0.05, -5, 11);
//        FTrobot.driveTrain.PivotTurn(-11, .035, true, 5);
        telemetry.addData("2.  After Impact:  ", " go to 2nd beacon");
        telemetry.update();
        sleep(1000);
        FTrobot.driveTrain.GoInches_lean(5, 0.5, 5, 1.5);
        int zero_red_level = robot.beaconSensorRt.red();
        int zero_blue_level = robot.beaconSensorRt.blue();
        telemetry.addData("3.  Starting white line:  ", " search");
        telemetry.update();
        sleep(1000);
        FTrobot.driveTrain.GoColor ("white", 0.5, 15);
        telemetry.addData("4.  At white line:  ", " check beacon");
        telemetry.update();
        sleep(5000);
        int first_red_level = robot.beaconSensorRt.red();
        int first_blue_level = robot.beaconSensorRt.blue();
        first_red_level -= zero_red_level;
        first_blue_level -= zero_blue_level;
        String BeaconColor1 = FTrobot.driveTrain.GetBeaconColorRt();
        telemetry.addData("Beacon Color", BeaconColor1);
        telemetry.addData("Red level:  ", first_red_level + " Blue: " + first_blue_level);
        telemetry.update();
        sleep (1000);
        if ((BeaconColor1.equals("blue") || BeaconColor1.equals("green"))
                && first_blue_level > 1.5 * first_red_level
                ) {
            telemetry.addData("5.  Saw beacon:  ", " blue");
            telemetry.update();
            sleep(1000);
            FTrobot.driveTrain.srvMove(1);
            sleep(200);
            FTrobot.driveTrain.GoInches(1, -0.18, 5);
            FTrobot.driveTrain.GoInches(2, 0.18, 5);
        }
        else {
            FTrobot.driveTrain.GoInches_lean(5.25, -.18, 5, -2);
            String BeaconColor = FTrobot.driveTrain.GetBeaconColorRt();
            int second_red_level = robot.beaconSensorRt.red();
            int second_blue_level = robot.beaconSensorRt.blue();
            second_red_level -= zero_red_level;
            second_blue_level -= zero_blue_level;
            telemetry.addData("Beacon Color", BeaconColor);
            telemetry.addData("Red level:  ", second_red_level + " Blue: " + second_blue_level);
            telemetry.update();
            if ((BeaconColor.equals("blue") || BeaconColor.equals("green"))
                    && (second_blue_level > 1.5 * second_red_level)
                    ) {
                telemetry.addData("5.  Next beacon:  ", " blue");
                telemetry.update();
                sleep(1000);
                FTrobot.driveTrain.srvMove(1);
                FTrobot.driveTrain.GoInches(1, 0.18, 5);
                FTrobot.driveTrain.GoInches(2, -0.18, 5);
            } else {
                telemetry.addData("6.  Never saw beacon:  ", " :(");
                telemetry.update();
                sleep(5000);
                FTrobot.driveTrain.stopMotors(false);
            }
        }
        */
    }
}
