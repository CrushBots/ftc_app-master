package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Red Right", group="Autonomous")
//@Disabled
public class Auto_RedRight extends Auto_CommonFunctions {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
        }

        // Step 1: Knock off the blue Jewel
        knockJewel(ALLIANCE_RED);

        // Step 4:
        //backwardDriveInches(24);
        robot.setDrivePower(-0.7,-0.7,-0.7,-0.7);
        sleep(2400);
        robot.setDrivePower(0.0,0.0,0.0,0.0);
        strafeRobot(0.7,"right");
        sleep(750);
        stopRobot();
        robot.setDrivePower(0.0,0.5,0.0,0.5);
        sleep(1500);
        robot.setDrivePower(0.0,0.0,0.0,0.0);
        robot.flopForward();
        robot.setDrivePower(0.7,0.7,0.7,0.7);
        sleep(1000);
        robot.setDrivePower(0.0,0.0,0.0,0.0);

        // Step 5: Turn towards crypto box
        //turnLeft(45);

        // Step 6: Drive forward
        //forwardDriveInches(22);

        // Step 7: place glyph in crypto box
        //openGlyphGrabber();

        // Step 8: Back up
        //backwardDriveInches(6);
    }
}
