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
FOR ANY DIRECT, INDIRE8CT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PurpleBot: Teleop Tank", group = "7079")
//@Disabled
public class PurpleBot_TeleOp_Iterative extends OpMode {

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
        robot.mtrElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    double current_tilt_position = 0;
    @Override
    public void loop() {
        float left_stick = -gamepad1.left_stick_y;
        float right_stick = -gamepad1.right_stick_y;
        float left_trigger = gamepad1.left_trigger;
        float right_trigger = gamepad1.right_trigger;
        double left_power;
        double right_power;
        float left_stick_abs = Math.abs (left_stick);
        float right_stick_abs = Math.abs (right_stick);
        if (left_stick_abs < 0.5) {
            left_power = 0.4 * left_stick_abs;
        } else {
            left_power = 0.2 + 1.6 * (left_stick_abs - 0.5);
        }
        if (right_stick_abs < 0.5) {
            right_power = 0.4 * right_stick_abs;
        } else {
            right_power = 0.2 + 1.6 * (right_stick_abs - 0.5);
        }

        // If sticks are in the same direction and about the same place, then driver probably just wants to go straight
        // so average the sticks with the average, so that they are more likely to go straight
        if ((left_stick > 0.05 && right_stick > 0.05
                || left_stick < -0.05 && right_stick < -0.05
            ) && Math.abs (left_stick - right_stick) < 0.2
                ) {
                double average_power = (left_power + right_power) / 2;
                left_power = (average_power + left_power) / 2;
                right_power = (average_power + right_power) / 2;
        }

        if (left_stick < 0) {
            left_power = -left_power;
        }
        if (right_stick < 0) {
            right_power = -right_power;
        }
        if (left_trigger > 0.2) {
            robot.mtrLeft.setPower(0.15 * left_trigger);
            robot.mtrRight.setPower(0.22 * left_trigger);
            if (left_trigger > .8) {
                robot.mtrLeft.setPower(0.55 * left_trigger);
                robot.mtrRight.setPower(0.8 * left_trigger);
            }
        } else if (right_trigger > 0.2) {
            robot.mtrLeft.setPower(0.22 * right_trigger);
            robot.mtrRight.setPower(0.15 * right_trigger);
            if (right_trigger > .8) {
                robot.mtrLeft.setPower(0.8 * right_trigger);
                robot.mtrRight.setPower(0.55 * right_trigger);
            }
        } else {
            robot.mtrLeft.setPower(left_power);
            robot.mtrRight.setPower(right_power);
        }
        if (gamepad1.left_bumper) {
            robot.srvPush.setPosition(0);
        } else if (gamepad1.right_bumper) {
            robot.srvPush.setPosition(1);
        } else if (gamepad1.x) {
            robot.srvPush.  setPosition(0.5);
        }
        if (gamepad2.right_bumper) {
            robot.srvCap.setPosition(0);
        } else if (gamepad2.right_trigger > 0.3){
            robot.srvCap.setPosition(0.65);
        } else if (gamepad2.a){
            robot.srvCap.setPosition(1);
        }
        double left_stick_x2 = gamepad2.left_stick_x;
        if (Math.abs (left_stick_x2) >= 0.05) {
            double new_tilt_position = current_tilt_position;
            if (left_stick_x2 < -0.05) {
                if (Math.abs(left_stick_x2) > current_tilt_position) {
                    new_tilt_position = Math.abs(left_stick_x2);
                }
            } else if (left_stick_x2 > 0.1) {
                if (left_stick_x2 >= 1 - current_tilt_position) {
                    new_tilt_position = 1 - left_stick_x2;
                }
            }
            robot.srvTilt.setPosition(new_tilt_position);
            current_tilt_position = new_tilt_position;
            telemetry.addData("Left_Stick_Current", left_stick_x2);
            telemetry.addData("Motor_Position", current_tilt_position);
            telemetry.update();
        }
        // Set elevator power
        double right_stick_y2 = gamepad2.right_stick_y;
        double right_stick_y2_abs = Math.abs (right_stick_y2);
        double elevator_power = 0;
        if (right_stick_y2_abs < 0.5) {
            elevator_power = 0.4 * right_stick_y2_abs;
        } else {
            elevator_power = 0.2 + 1.6 * (right_stick_y2_abs - 0.5);
        }
        if (right_stick_y2 < 0) {
            elevator_power = -elevator_power;
        }
        robot.mtrElevator.setPower(elevator_power);



        telemetry.addData("LeftPower:   ", left_power);
        telemetry.addData("RightPower:  ", right_power);
        telemetry.addData("LeftStick:   ", left_stick);
        telemetry.addData("RightStick:  ", right_stick);
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
