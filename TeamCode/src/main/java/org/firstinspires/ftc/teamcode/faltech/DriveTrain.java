package org.firstinspires.ftc.teamcode.faltech;

import android.content.res.Resources;
import android.graphics.Color;
import android.hardware.camera2.params.StreamConfigurationMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * Created by ddhol on 12/1/2015.
 */


public class DriveTrain {

    final static int pulsesPerRevolution = 1120;
    final static double tireCircumference = 12.56; //inches
    final static double veryLowPower = 0.05;
    final static double lowPower = 0.2;
    final static double powerUp = 1.0001;

    private LinearOpMode opMode;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private GyroSensor gyroSensor;
    private int last_heading;
    private int full_turns = 0;
    private double alpha_average = 0;
    private double red_average = 0;
    private double blue_average = 0;
    private double green_average = 0;
    private int color_count = 0;
    private ColorSensor TapeSensor;
    private ColorSensor BeaconColorSensor;
    //private Servo Beacon_Pusher;
    // float hsvValues[] = {0F,0F,0F};
    // final float values[] = hsvValues;


    public DriveTrain(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        opMode.telemetry.addData("DriveTrain", "Starting");
        // get hardware mappings
      gyroSensor.calibrate();
        // make sure the gyro is calibrated.
        while (gyroSensor.isCalibrating()) {
            Thread.sleep(100);
        }
        gyroSensor.resetZAxisIntegrator();
        while (gyroSensor.isCalibrating()) {
            Thread.sleep(100);
        }
        Thread.sleep(500);
        last_heading = 0;
        full_turns = 0;
//      opMode.waitForNextHardwareCycle();
        TapeSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
        BeaconColorSensor = opMode.hardwareMap.colorSensor.get("beaconSensor");
    }

    //stopMotors
    private void stopMotors(boolean fix_angle) throws InterruptedException {
        //int heading1 = getFullHeading();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        Thread.sleep(300);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        Thread.sleep(200);
        /*
        int heading2 = getFullHeading();
        opMode.telemetry.addData("Stop_Heading_Change", heading1 + " to " + heading2);
        if (heading1 == heading2) {
            //clean stop
        } else if (fix_angle) {
            //oops, one motor stopped too late and made robot turn.  Turn it back!
            Thread.sleep(600);
            PivotTurn(heading1 - heading2, 0.5, 5);
            Thread.sleep(100);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        Thread.sleep(300);
        if (fix_angle) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            Thread.sleep(300);
        }
        */
        opMode.telemetry.addData("StopMotors", "Complete");
  //    opMode.waitForNextHardwareCycle();
    }

    // getEncoderAverage
    private long getEncoderAverage() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
    }
    private int getFullHeading (){
        // this gives negatives for left angles (example: -1 instead of 359)
        // so going left always makes the angle smaller
        int heading = gyroSensor.getHeading();
        if (heading > 180 + last_heading) {
            full_turns = full_turns - 1;
        } else if (heading < last_heading - 180) {
            full_turns = full_turns + 1;
        }
        last_heading = heading;
        int full_heading = heading + full_turns * 360;
        opMode.telemetry.addData("FullHeading", full_heading);
        return (full_heading);
    }

    private String shorten (double number) {
        String num_string = String.valueOf(number);
        int len = num_string.length();
        if (len > 8) {
            num_string = num_string.substring(0,8);
        }
        return num_string;
    }

    private boolean Go(long number_of_steps, double normal_power, int timeout) throws InterruptedException {
        opMode.telemetry.addData("Go", number_of_steps);
        Thread.sleep(200);
        double power = lowPower;
        long startPositionLeft = (leftMotor.getCurrentPosition());
        long startPositionRight = (rightMotor.getCurrentPosition());
        long currentPositionLeft = startPositionLeft;
        long currentPositionRight = startPositionRight;
        double left_multiplier = 1;
        double right_multiplier = 1;
        // Reset gyro
        while (gyroSensor.isCalibrating()) {
            Thread.sleep(100);
        }
        gyroSensor.resetZAxisIntegrator();
        while (gyroSensor.isCalibrating()) {
            Thread.sleep(100);
        }
        Thread.sleep(350);
        last_heading = 0;
        full_turns = 0;
        // Done reset gyro
        int heading = getFullHeading();
        int startHeading = heading;
        // These keep track of where the gyro was last time it took over:
        long lastGyroStepsLeft = 0;
        long lastGyroStepsRight = 0;
        //
        ElapsedTime timer = new ElapsedTime();
        if (power > 0) { //go forward
            long currentStepsRight = 0;
            long currentStepsLeft = 0;

            while ((currentStepsRight < number_of_steps) && (timer.time() < timeout)) {
                currentPositionLeft = (leftMotor.getCurrentPosition());
                currentPositionRight = (rightMotor.getCurrentPosition());
                currentStepsLeft = Math.abs (currentPositionLeft - startPositionLeft);
                currentStepsRight = Math.abs (currentPositionRight - startPositionRight);
                /////////////////////////////////////////////////
                // Ramp up power at beginning with time
                // If we used distance, robot might never move if power is too low
                power = power + timer.time()*0.15;
                if (power >  normal_power){
                    power =  normal_power;
                }
                //////////////////////////////////////////////////
                // Ramp down power at end with remaining distance:
                // Can't use time because we don't know how much time is left
                double remaining_inches = (number_of_steps - currentStepsRight) / pulsesPerRevolution;
                // Start slowing down at this distance:
                double slow_inches = 10;
                if (remaining_inches < slow_inches) {
                    double slow_down_power =  normal_power * remaining_inches / slow_inches;
                    // Don't let power get too low or it might stop too soon!
                    if (slow_down_power < lowPower) {
                        slow_down_power = lowPower;
                    }
                    if (power > slow_down_power) {
                        power = slow_down_power;
                    }
                }
                //////////////////////////////
                heading = getFullHeading();
                double leftPower = power * left_multiplier;
                double rightPower = power * right_multiplier;
                if (heading == startHeading) {
                    //MOTOR position takeover
                    //going straight according to the gyro
                    //just make sure motors are turning the same steps
                    //find out how far the left and right motors have gone since the gyro last took over
                    long stepsFromLastGyroLeft = currentStepsLeft - lastGyroStepsLeft;
                    long stepsFromLastGyroRight = currentStepsRight - lastGyroStepsRight;
                    opMode.telemetry.addData("Gyro_Steps_L_R:  ", stepsFromLastGyroLeft + " " + stepsFromLastGyroRight);
                    if (stepsFromLastGyroLeft > stepsFromLastGyroRight) {
                        //Left motor is starting to go too fast.  Slow it down and speed up the other.
                        left_multiplier  = left_multiplier  / powerUp;
                        right_multiplier = right_multiplier * powerUp;
                    } else if (stepsFromLastGyroRight > stepsFromLastGyroLeft) {
                        //Right motor is starting to go too fast.  Slow it down and speed up the other.
                        left_multiplier  = left_multiplier  * powerUp;
                        right_multiplier = right_multiplier / powerUp;
                    } else {
                        left_multiplier = 1;
                        right_multiplier = 1;
                    }
                    opMode.telemetry.addData("Controller:  ", "Motor Position");
                    if (left_multiplier > 1) {
                        left_multiplier = 1;
                    }
                    if (right_multiplier > 1) {
                        right_multiplier = 1;
                    }
                    if (left_multiplier < 0.9) {
                        left_multiplier = 0.9;
                    }
                    if (right_multiplier < 0.9) {
                        right_multiplier = 0.9;
                    }
                    leftPower = power * left_multiplier;
                    rightPower = power * right_multiplier;
                } else {
                    //GYRO takeover
                    //drifting according to gyro, have it take over
                    if (heading > startHeading) {
                        //drifting right according to the gyro
                        //make the right motor go faster than the left
                        leftPower = 0.88 * power; //left_multiplier / powerUp;
                        rightPower = power; //right_multiplier * powerUp;
                    } else if (heading < startHeading) {
                        //drifting left according to the gyro
                        //make the left motor go faster than the right
                        leftPower = power; //left_multiplier * powerUp;
                        rightPower = 0.88 * power; //right_multiplier / powerUp;
                    }
                    //save off where the motors are while the gyro is in control of them
                    lastGyroStepsLeft = currentStepsLeft;
                    lastGyroStepsRight = currentStepsRight;
                    opMode.telemetry.addData("Controller:  ", "Gyro");
                }
                if (leftPower > 1) {
                    leftPower = 1;
                }
                if (rightPower > 1) {
                    rightPower = 1;
                }
                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);

                opMode.telemetry.addData("Heading was: ", String.valueOf(startHeading) + " now: " + String.valueOf(heading));
                opMode.telemetry.addData("Multipliers_L_R:  ", shorten(left_multiplier) + " " + shorten(right_multiplier));
                opMode.telemetry.addData("Power_L_R:  ", shorten(leftPower) + " " + shorten(rightPower));
                opMode.telemetry.addData("Position_L_R:  ", currentPositionLeft + " " + currentPositionRight);
                opMode.telemetry.addData("Steps_L_R:  ", currentStepsLeft + " " + currentStepsRight);
                opMode.telemetry.addData("StepTarget: ", number_of_steps);

            }
        } else if (power < 0) { //go backward
            long targetPosition = Math.abs(currentPositionRight - number_of_steps);
            while ((currentPositionRight > targetPosition) && (timer.time() < timeout)) {
                currentPositionRight = Math.abs(rightMotor.getCurrentPosition());
                //currentPosition = getEncoderAverage();
                opMode.telemetry.addData("Current: ", currentPositionRight);
                opMode.telemetry.addData("Target:  ", targetPosition);
            }
        }
        stopMotors(true);
        Thread.sleep(100);
        return timer.time() <= timeout;
    }
    public String GetBeaconColor () {
        return GetColor(BeaconColorSensor, false);
    }
    public String GetColor(ColorSensor SensorRGB, boolean check_white) {
        // looks for white if check_white is set
        //Color.RGBToHSV((SensorRGB.red() * 255) / 800, (SensorRGB.green() * 255) / 800, (SensorRGB.blue() * 255) / 800, hsvValues);
        int red = SensorRGB.red();
        int blue = SensorRGB.blue();
        int green = SensorRGB.green();
        int alpha = SensorRGB.alpha();
        int white = 0;
        if ( color_count == 0) {
            // Start setting average
            alpha_average = alpha;
            red_average = red;
            blue_average = blue;
            green_average = green;
        } else if ( color_count < 15) {
            // Then average the first few alphas to decide what number is gray from the field
            alpha_average = (alpha_average *  color_count + alpha) / ( color_count + 1);
            red_average = (red_average *  color_count + red) / ( color_count + 1);
            blue_average = (blue_average *  color_count + blue) / ( color_count + 1);
            green_average = (green_average *  color_count + green) / ( color_count + 1);

        } else {
            // Now the average (which is from the gray field) is already set
            if (alpha > 2 * alpha_average) {
                // if the alpha shoots up once we're past the average, then it must be white
                white = 1;
            }
        }
         color_count =  color_count + 1;
        opMode.telemetry.addData("red=  ", red + " average= " + red_average);
        opMode.telemetry.addData("blue= ", blue + " average= " + blue_average);
        opMode.telemetry.addData("green= ", green + " average= " + green_average);
        opMode.telemetry.addData("alpha= ", alpha + " average= " + alpha_average);
        double red_change = red - red_average;
        double blue_change = blue - blue_average;
        double green_change = green - green_average;

        if (check_white && white == 1) {
            return "white";
        } else if (red_change > blue_change && red_change > green_change) {
            return "red";
        } else if (blue_change > red_change && blue_change > green_change) {
            return "blue";
        } else if (green_change > blue_change && green_change > red_change) {
            return "green";
        }
        return "unknown_color";
    }

    public boolean GoColor(String target_color, double power, int timeout) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        long currentPosition = Math.abs(rightMotor.getCurrentPosition());
        leftMotor.setPower(lowPower);
        rightMotor.setPower(lowPower);
        boolean motors_on = true;
        double pause = timer.time() + 0.05;
        //long currentPosition = getEncoderAverage();
        //long currentPosition = Math.abs(rightMotor.getCurrentPosition());
        while (timer.time() < timeout) {
                if (timer.time() > pause) {
                    //turn motors on and off to make robot go very slow and stop on the line
                    if (motors_on) {
                        //if motors are on, turn them off
                        leftMotor.setPower(0.0);
                        rightMotor.setPower(0.0);
                        motors_on = false;
                    } else {
                        //if motors are off, turn them off
                        leftMotor.setPower(lowPower);
                        rightMotor.setPower(lowPower);
                        motors_on = true;
                    }
                    pause = timer.time() + 0.05;
                }
                boolean check_white = target_color.equals("white");
                String current_color = GetColor (TapeSensor, check_white);
                opMode.telemetry.addData("Color: ", current_color);
                opMode.telemetry.addData("Count: ",  color_count);
                if (current_color.equals(target_color)) {
                    stopMotors(true);
                }
        }
        stopMotors(true);
        return timer.time() <= timeout;
    }

    public boolean PrintColor(int timeout) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        while (timer.time() < timeout) {
            String current_color = GetColor (BeaconColorSensor, false);
            opMode.telemetry.addData("Color: ", current_color);
            opMode.telemetry.addData("Count: ",  color_count);
        }
        return timer.time() <= timeout;
    }

    //PivotRight
    private void PivotRight(double power, boolean in_place) {

        opMode.telemetry.addData("PivotRight Power", power);
        leftMotor.setPower(power);
        if (in_place) {
            rightMotor.setPower(-power);
        } else {
            rightMotor.setPower(0);
        }
    }

    //PivotLeft
    private void PivotLeft(double power, boolean in_place) {
        opMode.telemetry.addData("PivotLeft Power", power);
        if (in_place) {
            leftMotor.setPower(-power);
        } else {
            leftMotor.setPower(0);
        }
        rightMotor.setPower(power);
    }


    //GoInches
    public boolean GoInches(double inches, double power, int seconds) throws InterruptedException {
        opMode.telemetry.addData("GoInches", inches);
        long number_of_steps = (long) (((Math.abs(inches) / tireCircumference) * pulsesPerRevolution));
        return Go(number_of_steps, power, seconds);
    }

    //Pivot Turn
    public boolean PivotTurn(int degrees, double max_power, boolean in_place,int seconds) throws InterruptedException {
        opMode.telemetry.addData("PivotTurn", degrees);
        Thread.sleep(300);

        if ((degrees > 359) || (degrees < -359)) {
            opMode.telemetry.addData("Error", "Incorrect Degree Value" + degrees);
            return false;
        }


//        Thread.sleep(500);
        // Reset gyro
        while (gyroSensor.isCalibrating()) {
            Thread.sleep(100);
        }
        gyroSensor.resetZAxisIntegrator();
        while (gyroSensor.isCalibrating()) {
            Thread.sleep(100);
        }
        Thread.sleep(350);
        // Reset done
        last_heading = 0;
        full_turns = 0;
//        opMode.waitForNextHardwareCycle();

        ElapsedTime timer = new ElapsedTime();
        if (degrees != 0) { // turn
            opMode.telemetry.addData("Turn", degrees);
            double power = max_power * 0.02;
            if (degrees > 0) {
                PivotRight(power, in_place);
            } else {
                PivotLeft(power, in_place);
            }
            int heading = getFullHeading();
            int target_degrees = heading + degrees;
            boolean turn_done = false;
            while (!turn_done) {
                heading = getFullHeading();
                power = power + timer.time() * 0.3;
                if (power > max_power){
                    power = max_power;
                }
                // Ramp down power at end:
                int remaining_degrees = Math.abs (target_degrees - heading);
                // Start slowing down at this distance:
                double slow_degrees = 15;
                if (remaining_degrees < slow_degrees) {
                    double slow_down_power = max_power * remaining_degrees / slow_degrees;
                    // Don't let power get too low or it might stop too soon!
                    if (slow_down_power < lowPower) {
                        slow_down_power = lowPower;
                    }
                    if (power > slow_down_power) {
                        power = slow_down_power;
                    }
                }
                if (degrees > 0) {
                    if (heading > target_degrees) {
                        turn_done = true;
                        break;
                    } else {
                        PivotRight(power, in_place);
                    }
                } else {
                    if (heading < target_degrees) {
                        turn_done = true;
                        break;
                    } else {
                        PivotLeft(power, in_place);
                    }
                }
                if (timer.time() > seconds) {
                    stopMotors(false);
                    return false;
                }
                opMode.telemetry.addData("Heading", heading);
                opMode.telemetry.addData("Power", power);
            }
            stopMotors(false);
            opMode.telemetry.addData("PivotTurn Done", degrees);
            Thread.sleep(200);
        }
        return true;
    }

/*
    private boolean GoStraight(long distance, double power, int timeout) throws InterruptedException {
        double gain = .5;

        Thread.sleep(500);

        gyroSensor.resetZAxisIntegrator();// set heading to zero
        int heading = gyroSensor.getHeading();
        long currentPosition = Math.abs(rightMotor.getCurrentPosition());
        long targetPosition = Math.abs(currentPosition) + distance;
        //currentPosition = getEncoderAverage();
        ElapsedTime timer = new ElapsedTime();
        opMode.telemetry.addData("Pos: ", currentPosition);
        opMode.telemetry.addData("TPos: ", targetPosition);
        while(currentPosition < targetPosition && (timer.time() < timeout)) {
            if (power > 0 && heading == 0) {
                //go forward, equal power on both motors.
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            } else if (power > 0 && heading > 180) { //Drifting Left
                leftMotor.setPower(power + (.066 *(360 - heading) * gain));
                rightMotor.setPower(power - (.066 *(360 - heading) * gain));
            } else if (power > 0 && heading < 180 && heading != 0) { //Drifting Right
                leftMotor.setPower(power - (.066 * heading * gain));
                rightMotor.setPower(power + (.066 * heading * gain));
            }
            currentPosition = Math.abs(rightMotor.getCurrentPosition());
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        return timer.time() <= timeout;
    }

    public boolean GoStraitInches(double inches, double power, int timeout) throws InterruptedException{
        opMode.telemetry.addData("GoInches", inches);
        long distance = (long) (((Math.abs(inches) / tireCircumference) * pulsesPerRevolution));
        return GoStraight(distance, power, timeout);
    }
*/
    //Gyro Test
    public void GyroTest() {
        while (true) {
            opMode.telemetry.addData("Heading", gyroSensor.getHeading());
        }
    }

}