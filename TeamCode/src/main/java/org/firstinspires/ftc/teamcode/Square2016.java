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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.teamcode.faltech.FaltechRobot;

public class Square2016 {
    final static int pulsesPerRevolution = 1680;
    final static double tireCircumference = 12.56; //inches


    private LinearOpMode opMode;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private GyroSensor gyroSensor;
    public Square2016(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        opMode.telemetry.addData("DriveTrain", "constructor");

        // calibrate the gyro.
        gyroSensor.calibrate();
        // make sure the gyro is calibrated.
        while (gyroSensor.isCalibrating()) {
            Thread.sleep(50);
        }
        gyroSensor.resetZAxisIntegrator();
//      opMode.waitForNextHardwareCycle();
}
}

