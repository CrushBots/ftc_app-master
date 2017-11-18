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

        // Step 1: Grabbing the preloaded Glyph
        closeGlyphGrabber();

        // Step 2: Raise Glyph off ground
        raiseGlyph();

        // Step 3: Put down jewel arm
        lowerJewelArm();

        // Step 4: Read color of jewel and knock blue one off
        if (isRed(robot.sensorColor)){
            backwardDriveInches(-3);
            sleep(500);
            //forwardDriveInches(3);
        }
        else {
            if (isBlue(robot.sensorColor)) {
                forwardDriveInches(3);
                sleep(500);
                //backwardDriveInches(-3);
            }
        }

        // Step 5: Bring jewel arm back up
        raiseJewelArm();

        // Step 6: Drive into safe zone
        forwardDriveInches(36);

        // Step 7: Turn towards crypto box
        //robotRotate(0.8,"right");
        //sleep(1100);
        //robot.setDrivePower(0.0,0.0,0.0,0.0);


        // Step 8: Go Forward
        //forwardDriveInches(20);

        // Step 6: place glyph in crypto box
        //openGlyphGrabber();

        // Step 10: Back up
        //backwardDriveInches(5);
    }
}