package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Blue Right", group="Autonomous")
//@Disabled
public class Auto_BlueRight extends Auto_CommonFunctions {

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

        // Step 1: Knock off the Red Jewel
        knockJewel(ALLIANCE_BLUE);

        // Step 2: Read the Pictograph
        readPictograph();

        // Step 3: Move into front of the Cryptobox
        //switch (robot.vuMark) {
            //case LEFT:
                //backwardDriveInches(20);
                //break;
            //case RIGHT:
                //backwardDriveInches(30);
                //break;
            //default:
                //backwardDriveInches(40);
                //break;
        //}
        //forwardDriveInches(15);
        robot.setDrivePower(-0.7,-0.7,-0.7,-0.7);
        sleep(1500);
        robot.setDrivePower(0.0,0.0,0.0,0.0);
        //turnLeft(90);
        robot.setDrivePower(0.0,0.5,0.0,0.5);
        sleep(750);
        robot.setDrivePower(0.0,0.0,0.0,0.0);
        robot.flopForward();

        // Step 4: Release Glyphs
        //forwardDriveInches(10);
        robot.setDrivePower(-0.7,-0.7,-0.7,-0.7);
        sleep(1000);
        robot.setDrivePower(0.0,0.0,0.0,0.0);
        //backwardDriveInches(5);
    }
}
