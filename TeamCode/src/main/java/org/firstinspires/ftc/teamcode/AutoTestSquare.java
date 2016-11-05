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
    Autonomous program for Blue alliance, starting in Corner Side and finishing in the Safety.
TARGET SCORE:
    5

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.faltech.FaltechRobot;

/* This is a Autonomous for blue alliance,
It goes from the corner to the Blue Allience Repair Zone.
 */
public class AutoTestSquare extends LinearOpMode {

    private FaltechRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new FaltechRobot(this);
        // wait for the start button to be pressed
        waitForStart();

        //Auto Start
        /*
        robot.driveTrain.GoInches(20, .5, 15);
        robot.driveTrain.PivotTurn(45, .5, 15);
        robot.driveTrain.GoInches(20, .5, 5);
        robot.driveTrain.PivotTurn(45, .5, 15);
        robot.driveTrain.GoInches(20, .5, 5);
        robot.driveTrain.PivotTurn(45, .5, 15);
        robot.driveTrain.GoInches(20, .5, 5);
        robot.driveTrain.PivotTurn(45, .5, 15);
        robot.driveTrain.GoInches(20, .5, 5);
        robot.driveTrain.PivotTurn(45, .5, 15);
        robot.driveTrain.GoInches(20, .5, 5);
        robot.driveTrain.PivotTurn(45, .5, 15);
        robot.driveTrain.GoInches(20, .5, 5);
*/
    }
}
