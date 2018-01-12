package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Blue Left", group="Autonomous")
//@Disabled
public class Auto_BlueLeft extends Auto_CommonFunctions {

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

        backwardDriveInches(15);

        switch (robot.pictograph) {
            case LEFT:
                strafeRobot(0.7, "left");
                sleep(600);
            case RIGHT:
                strafeRobot(0.7, "left");
                sleep(1000);
            default:
                strafeRobot(0.7, "left");
                sleep(800);
        }

        // Step 4: Flop the Glyph upright
        robot.flopForward();

        // Step 5: Push the glyphs forward
        forwardDriveInches(7);

        // Step 5: Release the glyphs
        backwardDriveInches(3);

    }
}