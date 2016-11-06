package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PurpleBotHardware {
    /* Public OpMode members. */
    public DcMotor mtrLeft = null;
    public DcMotor mtrRight = null;
    public DcMotor mtrElevator = null;

    //Servos
    public Servo srvPush = null;
    public Servo srvTilt = null;

    //Sensors
    public GyroSensor groTurn = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public PurpleBotHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        mtrLeft = hwMap.dcMotor.get("mtrLeft");
        mtrLeft.setDirection(DcMotor.Direction.REVERSE);
        mtrLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrLeft.setPower(0);

        mtrRight = hwMap.dcMotor.get("mtrRight");
        mtrRight.setDirection(DcMotor.Direction.FORWARD);
        mtrRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrRight.setPower(0);

        mtrElevator = hwMap.dcMotor.get("mtrElevator");
        mtrElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrElevator.setPower(0);

        // Define and Initialize Servos
        srvPush = hwMap.servo.get("srvPush");
        srvTilt = hwMap.servo.get("srvTilt");


        groTurn = hwMap.gyroSensor.get("groTurn");
        groTurn.calibrate();
        while (groTurn.isCalibrating())
        { try {wait(500); } catch(Exception e){} }
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

