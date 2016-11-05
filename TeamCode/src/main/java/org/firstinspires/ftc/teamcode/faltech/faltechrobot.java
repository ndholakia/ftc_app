package org.firstinspires.ftc.teamcode.faltech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ddhol on 12/1/2015.
 */


public class FaltechRobot {

    private LinearOpMode opMode;

    public DriveTrain driveTrain;
//    public Arms arms;
//    public Collector collector;
//    public Climber_Savers climberSavers;

    public FaltechRobot(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        opMode.telemetry.addData("Robot", "contructor");
        // get hardware mappings

        driveTrain = new DriveTrain(opMode);
        //      arms = new Arms(opMode);
        // collector = new Collector(opMode);
        //climberSavers = new Climber_Savers(opMode);

//        driveTrain.churro_grabber.Up();
        //climberSavers.ClimberStartPosition();
    }

}
