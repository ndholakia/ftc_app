
/*
FALTECH 7079
FOUR POINTS MIDDLE SCHOOL
Thanks to Brendan Hollaway, from 6209 Venom
 */
package edu.fpms.faltech;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class FaltechTeleop_new extends OpMode {

	//Motors
	DcMotor MtrsLeft;
	DcMotor MtrsRight;
	//Motor Power Settings
	float MtrsLeftPower;
	float MtrsRightPower;

	public FaltechTeleop_new() {


	}

	/*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
	@Override
	public void init() {
		MtrsLeft = hardwareMap.dcMotor.get("MtrsLeft");
		MtrsRight = hardwareMap.dcMotor.get("MtrsRight");


	}


	/*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
	@Override
	public void loop() {
		float left_x = gamepad1.left_stick_x;
		float left_y = gamepad1.left_stick_y;
		float speed = left_y * left_y;
		float turn_speed = 0.5f * left_x * left_x;

		float left_speed;
		float right_speed;
		if (left_x >= 0) {
			left_speed =  speed + turn_speed;
			right_speed = speed - turn_speed;
		} else {
			left_speed =  speed - turn_speed;
			right_speed = speed + turn_speed;
		}
        if (left_y > 0) {
            left_speed = -left_speed;
            right_speed = -right_speed;
        }
		if (left_speed > 1) {
			left_speed = 1;
		}
		if (right_speed > 1) {
			right_speed = 1;
		}
        if (left_speed < -1) {
            left_speed = -1;
        }
        if (right_speed < -1) {
            right_speed = -1;
        }
        this.telemetry.addData("x", left_x);
        this.telemetry.addData("y", left_y);
        this.telemetry.addData("left_motor", left_speed);
        this.telemetry.addData("right_motor", right_speed);

		MtrsLeft.setPower(left_speed);
		MtrsRight.setPower(-right_speed);
		//Drive Train

				}

	/*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
	@Override
	public void stop() {

	}

}
