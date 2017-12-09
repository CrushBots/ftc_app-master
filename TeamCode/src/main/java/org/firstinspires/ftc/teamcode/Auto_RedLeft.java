package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Red Left", group="Autonomous")
//@Disabled
public class Auto_RedLeft extends Auto_CommonFunctions {

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
        knockJewel(ALLIANCE_RED);

        // Step 2: Read the Pictograph
        readPictograph();

        // Step 3: Move into front of the Cryptobox
        turnLeft(45);
        switch (robot.vuMark) {
            case LEFT:
                forwardDriveInches(20);
                break;
            case RIGHT:
                forwardDriveInches(30);
                break;
            default:
                forwardDriveInches(40);
                break;
        }
        turnRight(45);

        // Step 4: Release Glyphs
        forwardDriveInches(5);
        backwardDriveInches(5);

    }
}