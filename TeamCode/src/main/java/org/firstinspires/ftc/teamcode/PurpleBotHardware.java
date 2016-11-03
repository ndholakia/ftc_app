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

    //Servos
    public Servo srvPush = null;

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
        mtrRight = hwMap.dcMotor.get("mtrRight");

        srvPush = hwMap.servo.get("srvPush");

        groTurn = hwMap.gyroSensor.get("groTurn");
        mtrLeft.setDirection(DcMotor.Direction.REVERSE);
        mtrRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        mtrLeft.setPower(0);
        mtrRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        mtrLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

