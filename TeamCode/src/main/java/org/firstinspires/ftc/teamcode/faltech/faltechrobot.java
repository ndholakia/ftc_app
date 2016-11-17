package org.firstinspires.ftc.teamcode.faltech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PurpleBotHardware;

/**
 * Created by ddhol on 12/1/2015.
 */


public class FaltechRobot {

    private LinearOpMode opMode;
    private PurpleBotHardware robot;

    public DriveTrain driveTrain;
//    public Arms arms;
//    public Collector collector;
//    public Climber_Savers climberSavers;

    public FaltechRobot(LinearOpMode opMode, PurpleBotHardware robot) throws InterruptedException {
        this.opMode = opMode;
        this.robot = robot;
        opMode.telemetry.addData("Robot", "constructor");
        // get hardware mappings

        driveTrain = new DriveTrain(opMode, robot);
        //      arms = new Arms(opMode);
        // collector = new Collector(opMode);
        //climberSavers = new Climber_Savers(opMode);

//        driveTrain.churro_grabber.Up();
        //climberSavers.ClimberStartPosition();
    }

}
