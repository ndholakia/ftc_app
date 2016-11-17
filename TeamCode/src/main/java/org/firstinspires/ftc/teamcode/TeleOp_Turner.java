/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PurpleBot: Teleop Turner", group = "7079")
//@Disabled
public class TeleOp_Turner extends OpMode {

    /* Declare OpMode members. */
    PurpleBotHardware robot = new PurpleBotHardware(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        robot.mtrLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("status:", "Ready Player One");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        float speed_stick = -gamepad1.left_stick_y;
        float steer_stick = gamepad1.right_stick_x;
        float left_trigger = gamepad1.left_trigger;
        float right_trigger = gamepad1.right_trigger;
        double speed_power;
        double steer_power;
        float speed_stick_abs = Math.abs (speed_stick);
        float steer_stick_abs = Math.abs (steer_stick);
        if (speed_stick_abs < 0.5) {
            speed_power = 0.4 * speed_stick_abs;
        } else {
            speed_power = 0.2 + 1.6 * (speed_stick_abs - 0.5);
        }
        if (steer_stick_abs < 0.5) {
            steer_power = 0.4 * steer_stick_abs;
        } else {
            steer_power = 0.2 + 1.6 * (steer_stick_abs - 0.5);
        }
        double left_power;
        double right_power;
        if (steer_stick >= 0) {
            left_power =  speed_power + steer_power;
            right_power = speed_power - steer_power;
        } else {
            left_power =  speed_power - steer_power;
            right_power = speed_power + steer_power;
        }
        if (speed_stick < 0) {
            left_power = -left_power;
            right_power = -right_power;
        }
        if (left_power > 1) {
            left_power = 1;
        }
        if (left_power < -1) {
            left_power = -1;
        }
        if (right_power > 1) {
            right_power = 1;
        }
        if (right_power < -1) {
            right_power = -1;
        }
        if (left_trigger > 0.2) {
            robot.mtrLeft.setPower(0.23 * left_trigger);
            robot.mtrRight.setPower(0.15 * left_trigger);
        } else if (right_trigger > 0.2) {
            robot.mtrLeft.setPower(0.23 * right_trigger);
            robot.mtrRight.setPower(0.15 * right_trigger);
        } else {
            robot.mtrLeft.setPower(left_power);
            robot.mtrRight.setPower(right_power);
        }
        telemetry.addData("LeftPower:   ", left_power);
        telemetry.addData("RightPower:  ", right_power);
        telemetry.addData("SpeedStick:  ", speed_stick);
        telemetry.addData("SteerStick:  ", steer_stick);
        telemetry.addData("LeftTrig:    ", left_trigger);
        telemetry.addData("RightTrig:   ", right_trigger);
        telemetry.addData("LeftEncode:  ", robot.mtrLeft.getCurrentPosition());
        telemetry.addData("RightEncode: ", robot.mtrRight.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
