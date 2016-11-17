package org.firstinspires.ftc.teamcode.faltech;

import android.content.res.Resources;
import android.graphics.Color;
import android.hardware.camera2.params.StreamConfigurationMap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PurpleBotHardware;


/**
 * Created by ddhol on 12/1/2015.
 */


    public class DriveTrain {
    /* Declare OpMode members. */
    private PurpleBotHardware robot;

    final static int pulsesPerRevolution = 1600;
    //Should be 1792
    final static double tireCircumference = 12.56; //inches
    final static double veryLowPower = 0.15;
    final static double lowPower = 0.25;
    final static double powerUp = 1.0001;

    private LinearOpMode opMode;
//    private DcMotor leftMotor;
//    private DcMotor rightMotor;
//    private GyroSensor robot.groTurn;

    private int last_heading;
    private int full_turns = 0;
    private double alpha_average = 0;
    private double red_average = 0;
    private double blue_average = 0;
    private double green_average = 0;
    private int color_count = 0;
//    private ColorSensor TapeSensor;
//    private ColorSensor // ColorSensor;
    //private Servo Beacon_Pusher;
    // float hsvValues[] = {0F,0F,0F};
    // final float values[] = hsvValues;


    public DriveTrain(LinearOpMode opMode, PurpleBotHardware robot) throws InterruptedException {
        this.opMode = opMode;
        this.robot = robot;
        opMode.telemetry.addData("DriveTrain", "Starting");
//        leftMotor = opMode.hardwareMap.dcMotor.get("MtrsLeft");
//        rightMotor = opMode.hardwareMap.dcMotor.get("MtrsRight");
//        robot.mtrLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.mtrRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.mtrRight.setDirection(DcMotor.Direction.REVERSE);

        // Gyro sensor
//        gyroSensor = opMode.hardwareMap.gyroSensor.get("gyroSensor");
        robot.mtrLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       /* robot.groTurn.calibrate();
        while (robot.groTurn.isCalibrating()) {
            Thread.sleep (50);
        }
        robot.groTurn.resetZAxisIntegrator();
        while (robot.groTurn.isCalibrating()) {
            Thread.sleep(50);
        }
        Thread.sleep(100); */


        last_heading = 0;
        full_turns = 0;

        // Color sensors
//        TapeSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
//        BeaconColorSensor = opMode.hardwareMap.colorSensor.get("beaconSensor");
        robot.tapeSensor.enableLed(true);
        robot.tapeSensor.setI2cAddress(I2cAddr.create8bit(0x48));
        robot.beaconSensorLt.setI2cAddress(I2cAddr.create8bit(0x42));
        robot.beaconSensorRt.setI2cAddress(I2cAddr.create8bit(0x44));
    }

    public void SuperAuto(String color, boolean delay_start, int beacon_count, boolean center) throws InterruptedException {
        //direction = 1 for right , -1 for left (red)
        robot.mtrLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrLeft.setMaxSpeed(2240);
        robot.mtrRight.setMaxSpeed(2240);
        robot.srvCap.setPosition(1);
        ColorSensor beaconSensor;
        int direction;
        if (color.equals("blue")) {
            direction = 1;
            beaconSensor = robot.beaconSensorRt;
        } else if (color.equals("red")) {
            direction = -1;
            beaconSensor = robot.beaconSensorLt;
        } else {
            direction = 1;  //BAD COLOR!!
            opMode.telemetry.addData("Bad COLOR!!!", color);
            opMode.telemetry.update();
            Thread.sleep(10000);
            return;
        }
        srvMove(-direction, false);
        if (delay_start) {
            Thread.sleep(4000);
            GoInches(30, 0.75, 5);
        } else {
            GoInches(6, .6, 6);
        }
        /*
        int angle = 28;
        if (direction == -1) {
            angle -= 3;
        }
        PivotTurn(-angle * direction, .65, false, 5);
        opMode.telemetry.addData("1.  Starting:  ", " long angle");
        opMode.telemetry.update();

        //Thread.sleep(1000);
        */

        GoInches(67, 0.8, 8);
        GoImpact(0.6, -2 * direction, 10);
//        FTrobot.driveTrain.PivotTurn(-11, .035, true, 5);
        opMode.telemetry.addData("2.  After Impact:  ", " go to 2nd beacon");
        opMode.telemetry.update();
        //Thread.sleep(1000);
        GoInches_lean(4, 0.7, 5, 1.8 * direction, false);
        int zero_red_level = beaconSensor.red();
        int zero_blue_level = beaconSensor.blue();
        opMode.telemetry.addData("3.  Starting white line:  ", " search");
        opMode.telemetry.update();
        //Thread.sleep(1000);
        GoColor ("white", 0.6, 10, 1.4*direction);
        opMode.telemetry.addData("4.  At white line:  ", " check beacon");
        opMode.telemetry.update();
        //        Thread.sleep(5000);
        find_beacon (beaconSensor, color, direction, zero_red_level, zero_blue_level, false);
        if (beacon_count == 2) {
            double lean_multiplier = 1;
            if (direction == -1) {
                lean_multiplier = 1.5;
            }
            if (direction == -1) {
                //int pivot_angle = -1 * direction;
                //PivotTurn(pivot_angle, .5, true, 1);
            }
            GoInches_lean(40, -0.7, 6, 2 * direction, false);
            GoColor ("white", -0.65, 10, 1.2 * direction);
            find_beacon (beaconSensor, color, direction, zero_red_level, zero_blue_level, true);
        }
        if (center) {
            GoInches(36, 0.9, 7);
            PivotTurn(-direction *(direction == 1 ? 112 : 115) , 0.7, false, 5);
            GoInches(42, 1, 3);
            GoInches(6, -1, 5);
            //GoInches(9, 1, 3);
            GoInches_lean(9, 1, 3, 1.7*direction, true);

        }
    }

    public void find_beacon (ColorSensor beaconSensor, String color, int direction, int zero_red_level, int zero_blue_level, boolean going_backward) throws InterruptedException {
        int first_red_level = beaconSensor.red();
        int first_blue_level = beaconSensor.blue();
        first_red_level -= zero_red_level;
        first_blue_level -= zero_blue_level;
        String BeaconColor1 = GetBeaconColor(beaconSensor);
        opMode.telemetry.clear();
        opMode.telemetry.addData("Beacon Color1", BeaconColor1);
        opMode.telemetry.addData("Want Color", color);
        opMode.telemetry.addData("Red level:  ", first_red_level + " Blue: " + first_blue_level);
        opMode.telemetry.update();
        //Thread.sleep (3000);
        String color2;
        if ((BeaconColor1.equals(color))
                && (color.equals("blue")
                        ? (first_blue_level > 1.5 * first_red_level)
                        : (first_red_level > 1.5 * first_blue_level)
                    )
                ) {
            opMode.telemetry.addData("5.  Saw beacon:  ", BeaconColor1);
            opMode.telemetry.update();
            //            Thread.sleep(1000);
            push_button (going_backward, direction);
        } else {
            GoInches_lean(4.75, -.4, 5, 2 , false);
            String BeaconColor = GetBeaconColor(beaconSensor);
            int second_red_level = beaconSensor.red();
            int second_blue_level = beaconSensor.blue();
            second_red_level -= zero_red_level;
            second_blue_level -= zero_blue_level;
            opMode.telemetry.clear();
            opMode.telemetry.addData("Beacon Color2", BeaconColor);
            opMode.telemetry.addData("Want Color", color);
            opMode.telemetry.addData("Red level:  ", second_red_level + " Blue: " + second_blue_level);
            opMode.telemetry.update();
            //Thread.sleep (3000);
            if ((BeaconColor.equals(color))
                    && (color.equals("blue")
                            ? (second_blue_level > 1.5 * second_red_level)
                            : (second_red_level > 1.5 * second_blue_level)
                        )
                    ) {
                opMode.telemetry.addData("5.  Next beacon:  ", " blue");
                opMode.telemetry.update();
                //Thread.sleep(1000);
                push_button (true,direction);
            } else {
                opMode.telemetry.addData("6.  Never saw beacon:  ", " :(");
                opMode.telemetry.update();
                //Thread.sleep(5000);
                stopMotors(false);
            }
        }
        srvMove(-direction, false);
    }

    public void push_button (boolean going_backward,int direction) throws InterruptedException {
        srvMove(direction, true);
        if (going_backward) {
            PivotTurn(3 * direction, 0.3, true, 1);
        }
        Thread.sleep(50);

        if (going_backward) {
            GoInches_lean(1.1, -.5, 1, -2.5 * direction, true);
            GoInches_lean(4, .5, 4, 1.5 * direction, true);
            if (direction == 1) {
                GoInches_lean(2, -.5, 1, -2 * direction, true);
                GoInches_lean(2, .5, 1, 2 * direction, true);
            }
        } else {
            GoInches_lean(2, -.5, 1, -3 * direction, true);
            GoInches_lean(3.35, .5, 2, 3 * direction, true);
        }
        srvMove(-direction, false);

    }

    //stopMotors
    public void stopMotors(boolean fix_angle) throws InterruptedException {
        //int heading1 = getFullHeading();
        robot.mtrLeft.setPower(0);
        robot.mtrRight.setPower(0);
        Thread.sleep(50);
        robot.mtrLeft.setPower(0);
        robot.mtrRight.setPower(0);
        Thread.sleep(50);
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

    private int getFullHeading (){
        // this gives negatives for left angles (example: -1 instead of 359)
        // so going left always makes the angle smaller
        int heading = robot.groTurn.getHeading();
        if (heading > 180 + last_heading) {
            full_turns = full_turns - 1;
        } else if (heading < last_heading - 180) {
            full_turns = full_turns + 1;
        }
        last_heading = heading;
        int full_heading = heading + full_turns * 360;
        opMode.telemetry.addData("FullHeading", full_heading);
        opMode.telemetry.update();
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

    public boolean GoShort(long inches, double normal_power) throws InterruptedException {
        Thread.sleep (250);
        long number_of_steps = (long) (((Math.abs(inches) / tireCircumference) * pulsesPerRevolution));
        long startPositionRight = (robot.mtrRight.getCurrentPosition());
        long currentPositionRight = startPositionRight;
        long currentStepsRight = 0;
        while ((currentStepsRight < number_of_steps)) {
            currentPositionRight = (robot.mtrRight.getCurrentPosition());
            currentStepsRight = Math.abs (currentPositionRight - startPositionRight);
            robot.mtrLeft.setPower(normal_power);
            robot.mtrRight.setPower(normal_power);
        }
        stopMotors(true);
        return true;
    }


    private boolean Go(long number_of_steps, double normal_power, int timeout, double lean, boolean pulsate) throws InterruptedException {
        opMode.telemetry.addData("Go", number_of_steps);
        double power = lowPower;
        double normal_power_abs = Math.abs(normal_power);
        long startPositionLeft = (robot.mtrLeft.getCurrentPosition());
        long startPositionRight = (robot.mtrRight.getCurrentPosition());
        long currentPositionLeft = startPositionLeft;
        long currentPositionRight = startPositionRight;
        double left_multiplier = 1;
        double right_multiplier = 1;
        // Reset gyro
        while (robot.groTurn.isCalibrating()) {
            Thread.sleep(50);
        }
        robot.groTurn.resetZAxisIntegrator();
        while (robot.groTurn.isCalibrating()) {
            Thread.sleep(50);
        }
        Thread.sleep(50);
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
        double pause = timer.time() + 0.05;
        boolean motors_on = true;

        long currentStepsRight = 0;
        long currentStepsLeft = 0;

        while ((currentStepsRight < number_of_steps) && (timer.time() < timeout)) {
            currentPositionLeft = (robot.mtrLeft.getCurrentPosition());
            currentPositionRight = (robot.mtrRight.getCurrentPosition());
            currentStepsLeft = Math.abs (currentPositionLeft - startPositionLeft);
            currentStepsRight = Math.abs (currentPositionRight - startPositionRight);
            /////////////////////////////////////////////////
            // Ramp up power at beginning with time
            // If we used distance, robot might never move if power is too low
            power = power + timer.time()*0.7;
            if (power >  normal_power_abs){
                power =  normal_power_abs;
            }
            //////////////////////////////////////////////////
            // Ramp down power at end with remaining distance:
            // Can't use time because we don't know how much time is left
            double remaining_inches = (number_of_steps - currentStepsRight) / pulsesPerRevolution;
            // Start slowing down at this distance:
            double slow_inches = 10;
            if (remaining_inches < slow_inches) {
                double slow_down_power =  normal_power_abs * remaining_inches / slow_inches;
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
            if (lean != 0) {
                if (Math.abs(heading - startHeading) > 25) {
                    if (lean > 1) {
                        lean = 1.2;
                    } else if (lean < -1) {
                        lean = -1.2;
                    }
                }
            }
            if (lean > 0) {
                leftPower *= lean;
                rightPower /= lean;
            }
            if (lean < 0) {
                leftPower /= Math.abs(lean);
                rightPower *= Math.abs(lean);
            }
            if (leftPower > 1) {
                leftPower = 1;
            }
            if (rightPower > 1) {
                rightPower = 1;
            }
            double direction;
            if (normal_power < 0) {
                direction = -1;
            } else {
                direction = 1;
            }

/////////////////////////
            double mtrLeftPower = (direction * leftPower);
            double mtrRightPower = (direction * rightPower);
            if (pulsate) {
                //long currentPosition = getEncoderAverage();
                //long currentPosition = Math.abs(rightMotor.getCurrentPosition());
                int start_heading = getFullHeading();
                if (timer.time() > pause) {
                    //turn motors on and off to make robot go very slow and stop on the line
                    if (motors_on) {
                        //if motors are on, turn them off
                        robot.mtrLeft.setPower(0.0);
                        robot.mtrRight.setPower(0.0);
                        motors_on = false;
                    } else {
                        //if motors are off, turn them off
                        robot.mtrLeft.setPower(mtrLeftPower);
                        robot.mtrRight.setPower(mtrRightPower);
                        motors_on = true;
                    }
                    pause = timer.time() + 0.05;
                }
            } else {
                robot.mtrLeft.setPower(mtrLeftPower);
                robot.mtrRight.setPower(mtrRightPower);
            }
            opMode.telemetry.addData("Heading was: ", String.valueOf(startHeading) + " now: " + String.valueOf(heading));
            opMode.telemetry.addData("Multipliers_L_R:  ", shorten(left_multiplier) + " " + shorten(right_multiplier));
            opMode.telemetry.addData("Power_L_R:  ", shorten(leftPower) + " " + shorten(rightPower));
            opMode.telemetry.addData("Position_L_R:  ", currentPositionLeft + " " + currentPositionRight);
            opMode.telemetry.addData("Steps_L_R:  ", currentStepsLeft + " " + currentStepsRight);
            opMode.telemetry.addData("StepTarget: ", number_of_steps);
        }
        stopMotors(true);
        //Thread.sleep(50);
        return timer.time() <= timeout;
    }
    public String GetBeaconColorLt () {
        return GetColor(robot.beaconSensorLt, false, true);
    }
    public String GetBeaconColorRt () {
        return GetColor(robot.beaconSensorRt, false, true);
    }
    public String GetBeaconColor (ColorSensor beaconSensor) {
        return GetColor(beaconSensor, false, true);
    }

    public String GetColor(ColorSensor SensorRGB, boolean check_white, boolean print_color) {
        // looks for white if check_white is set
        //Color.RGBToHSV((SensorRGB.red() * 255) / 800, (SensorRGB.green() * 255) / 800, (SensorRGB.blue() * 255) / 800, hsvValues);
        int red = SensorRGB.red();
        int blue = SensorRGB.blue();
        int green = SensorRGB.green();
        int alpha = SensorRGB.alpha();
        int white = 0;
        if (check_white) {
            if (color_count == 0) {
                // Start setting average
                alpha_average = alpha;
                red_average = red;
                blue_average = blue;
                green_average = green;
            } else if (color_count < 15) {
                // Then average the first few alphas to decide what number is gray from the field
                alpha_average = (alpha_average * color_count + alpha) / (color_count + 1);
                red_average = (red_average * color_count + red) / (color_count + 1);
                blue_average = (blue_average * color_count + blue) / (color_count + 1);
                green_average = (green_average * color_count + green) / (color_count + 1);
            } else {
                // Now the average (which is from the gray field) is already set
                if (alpha > 2 + 2 * alpha_average) {
                    // if the alpha shoots up once we're past the average, then it must be white
                    white = 1;
                }
            }
            color_count = color_count + 1;
        }
        if (print_color) {
            opMode.telemetry.addData("red=  ", red + " average= " + red_average);
            opMode.telemetry.addData("blue= ", blue + " average= " + blue_average);
            opMode.telemetry.addData("green= ", green + " average= " + green_average);
            opMode.telemetry.addData("alpha= ", alpha + " average= " + alpha_average);
            opMode.telemetry.update();
        }
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

    public boolean GoImpact(double power, int angle, int timeout) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        opMode.telemetry.addData("GoImpact: ", "on");
        opMode.telemetry.update();
        Thread.sleep(50);
        robot.mtrLeft.setPower(lowPower);
        robot.mtrRight.setPower(lowPower);
        boolean motors_on = true;
        double pause = timer.time() + 0.05;
        //long currentPosition = getEncoderAverage();
        //long currentPosition = Math.abs(rightMotor.getCurrentPosition());
        int start_heading = getFullHeading();
        while (timer.time() < timeout) {
            if (timer.time() > pause) {
                //turn motors on and off to make robot go very slow and stop on the line
                if (motors_on) {
                    //if motors are on, turn them off
                    robot.mtrLeft.setPower(0.0);
                    robot.mtrRight.setPower(0.0);
                    motors_on = false;
                } else {
                    //if motors are off, turn them off
                    robot.mtrLeft.setPower(lowPower);
                    robot.mtrRight.setPower(lowPower);
                    motors_on = true;
                }
                pause = timer.time() + 0.05;
            }
            int heading = getFullHeading();
            opMode.telemetry.addData("Headings: ", start_heading + " to " + heading);
            opMode.telemetry.update();
            if (Math.abs(heading - start_heading) >= 4) {
                opMode.telemetry.addData("Impact Detected!", " Turn now");
                opMode.telemetry.update();
                if (angle != 0) {
                    PivotTurn (angle, .5, false, 3);
                }
                break;
            }
        }
        stopMotors(true);
        return timer.time() <= timeout;
    }

    public boolean GoColor(String target_color, double power, int timeout, double lean) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        long currentPosition = Math.abs(robot.mtrRight.getCurrentPosition());
        opMode.telemetry.addData("GoColor: ", target_color);
        opMode.telemetry.update();
        Thread.sleep(50);
        robot.mtrLeft.setPower(power);
        robot.mtrRight.setPower(power);
        boolean motors_on = true;
        double pause = timer.time() + 0.05;
        //long currentPosition = getEncoderAverage();
        //long currentPosition = Math.abs(rightMotor.getCurrentPosition());
        while (timer.time() < timeout) {
                if (timer.time() > pause) {
                    //turn motors on and off to make robot go very slow and stop on the line
                    if (motors_on) {
                        //if motors are on, turn them off
                        robot.mtrLeft.setPower(0.0);
                        robot.mtrRight.setPower(0.0);
                        motors_on = false;
                        pause = timer.time() + 0.05;
                    } else {
                        //if motors are off, turn them off
                        double leftPower = Math.abs(power);
                        double rightPower = Math.abs(power);
                        if (lean > 0) {
                            leftPower *= lean;
                            rightPower /= lean;
                        }
                        if (lean < 0) {
                            leftPower /= Math.abs(lean);
                            rightPower *= Math.abs(lean);
                        }
                        if (leftPower > 1) {
                            leftPower = 1;
                        }
                        if (rightPower > 1) {
                            rightPower = 1;
                        }
                        double direction;
                        if (power < 0) {
                            direction = -1;
                        } else {
                            direction = 1;
                        }

                        robot.mtrLeft.setPower(direction * leftPower);
                        robot.mtrRight.setPower(direction * rightPower);
                        motors_on = true;
                        pause = timer.time() + 0.05;
                    }
                }
                boolean check_white = target_color.equals("white");
                String current_color = GetColor (robot.tapeSensor, check_white, true);
                //opMode.telemetry.addData("GoColor:      ", target_color);
                //opMode.telemetry.addData("CurrentColor: ", current_color);
                //opMode.telemetry.update();
                if (current_color.equals(target_color)) {
                    stopMotors(true);
                    break;
                }
        }
        if (timer.time() >= timeout) {
            stopMotors(true);
        }
        return timer.time() <= timeout;
    }

    public boolean PrintColor(int timeout) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        while (timer.time() < timeout) {
            String beacon_color = GetColor (robot.beaconSensorLt, false, false);
            String tape_color = GetColor (robot.tapeSensor, true, false); //true for check_white
            opMode.telemetry.addData("Beacon: ", beacon_color);
            opMode.telemetry.addData("Tape:   ", tape_color);
            opMode.telemetry.addData("Count:  ",  color_count);
            opMode.telemetry.update();
        }
        return timer.time() <= timeout;
    }

    public boolean PrintBeaconColor(int timeout) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        while (timer.time() < timeout) {
            String beacon_color = GetColor (robot.beaconSensorLt, false, true);
        }
        return timer.time() <= timeout;
    }
    public void srvMove(int direction, boolean push) {
        if (direction > 0) {
            robot.srvPush.setPosition( push ? 0.75 : 1);
        } else if (direction < 0) {
            robot.srvPush.setPosition(push ? 0.25 : 0);
        }
    }
    //PivotRight
    private void PivotRight(double power, boolean in_place) {
        opMode.telemetry.addData("PivotRight Power", power);
        robot.mtrLeft.setPower(power);
        if (in_place) {
            robot.mtrRight.setPower(-power);
        } else {
            robot.mtrRight.setPower(0);
        }
    }

    //PivotLeft
    private void PivotLeft(double power, boolean in_place) {
        opMode.telemetry.addData("PivotLeft Power", power);
        robot.mtrRight.setPower(power);
        if (in_place) {
            robot.mtrLeft.setPower(-power);
        } else {
            robot.mtrLeft.setPower(0);
        }
    }


    //GoInches
    public boolean GoInches(double inches, double power, int seconds) throws InterruptedException {
        return GoInches_lean (inches, power, seconds, 0, false);
    }
    public boolean GoInches_lean (double inches, double power, int seconds, double lean, boolean pulsate) throws InterruptedException {
        opMode.telemetry.addData("GoInches", inches);
        long number_of_steps = (long) (((Math.abs(inches) / tireCircumference) * pulsesPerRevolution));
        return Go(number_of_steps, power, seconds, lean, pulsate);
    }

    //Pivot Turn
    public boolean PivotTurn(int degrees, double max_power, boolean in_place,int seconds) throws InterruptedException {
        opMode.telemetry.addData("PivotTurn", degrees);
        opMode.telemetry.update();
        Thread.sleep(50);

        if ((degrees > 359) || (degrees < -359)) {
            opMode.telemetry.addData("Error", "Incorrect Degree Value" + degrees);
            return false;
        }


//        sleep(500);
        // Reset gyro
        while (robot.groTurn.isCalibrating()) {
            Thread.sleep(50);
        }
        robot.groTurn.resetZAxisIntegrator();
        while (robot.groTurn.isCalibrating()) {
            Thread.sleep(50);
        }
        Thread.sleep(50);
        // Reset done
        last_heading = 0;
        full_turns = 0;
//        opMode.waitForNextHardwareCycle();

        ElapsedTime timer = new ElapsedTime();
        if (degrees != 0) { // turn
            opMode.telemetry.addData("Turn", degrees);
            double power = 0.2;
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
                power = power + timer.time() * (max_power > 0.6 ? 1 : 0.25);
                if (power > max_power){
                    power = max_power;
                }
                // Ramp down power at end:
                int remaining_degrees = Math.abs (target_degrees - heading);
                // Start slowing down at this distance:
                double slow_degrees = max_power > .6 ? 3 : 8;
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
                opMode.telemetry.update();
            }
            stopMotors(false);
            opMode.telemetry.addData("PivotTurn Done", degrees);
            opMode.telemetry.update();
            Thread.sleep(100);
        }
        return true;
    }

/*
    private boolean GoStraight(long distance, double power, int timeout) throws InterruptedException {
        double gain = .5;

        sleep(500);

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
            opMode.telemetry.addData("Heading", robot.groTurn.getHeading());
            opMode.telemetry.update();
        }
    }

}