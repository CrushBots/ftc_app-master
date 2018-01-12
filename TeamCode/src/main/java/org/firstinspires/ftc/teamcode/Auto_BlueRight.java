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

        switch (robot.pictograph) {
            case LEFT:
                forwardDriveInches(12);
            case RIGHT:
                forwardDriveInches(20);
            default:
                forwardDriveInches(16);
        }

        // Step 4: Turn towards the cryptobox
        turnRight(90);

        // Step 5: Flop the Glyph upright
        robot.flopForward();

        // Step 6: Push the Glyphs forward
        forwardDriveInches(7);

        // Step 7: Release the glyphs
        backwardDriveInches(3);
    }
}
