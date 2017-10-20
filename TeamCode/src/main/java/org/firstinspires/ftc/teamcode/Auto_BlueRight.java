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

        robot.rightGlyphServo.setPosition(robot.RIGHT_GLYPH_SERVO_CLOSED_POS);
        robot.leftGlyphServo.setPosition(robot.LEFT_GLYPH_SERVO_CLOSED_POS);
        sleep(400);

        // Step 1: Raise Glyph off ground
        raiseGlyph();

        // Step 2: Put down jewel arm
        lowerJewelArm();

        // Step 3: Read color of jewel and knock blue one off
        if (isBlue(robot.sensorColor)){
            robot.setDrivePower(-0.5,-0.5);
            sleep(250);
            robot.setDrivePower(0.0,0.0);
            robot.setDrivePower(0.5,0.5);
            sleep(250);
            robot.setDrivePower(0.0,0.0);

        }
        else {
            if (isRed(robot.sensorColor)) {
                robot.setDrivePower(0.5,0.5);
                sleep(250);
                robot.setDrivePower(0.0,0.0);
                robot.setDrivePower(-0.5,-0.5);
                sleep(250);
                robot.setDrivePower(0.0,0.0);

            }
        }

        sleep (250);
        robot.setDrivePower(0.0,0.0);

        // Step 4: Bring jewel arm back up
        raiseJewelArm();

        // Step 5: Drive into safe zone
        robot.setDrivePower(-0.4,-0.4);
        sleep(2000);
        robot.setDrivePower(0.0,0.0);

        // Step 6: Turn towards crypto box
        robot.setDrivePower(0.5,-0.5);
        sleep(1500);
        robot.setDrivePower(0.0,0.0);

        // Step 8: place glyph in crypto box
        lowerGlyph();
        robot.leftGlyphServo.setPosition(robot.LEFT_GLYPH_SERVO_OPEN_POS);
        robot.rightGlyphServo.setPosition(robot.RIGHT_GLYPH_SERVO_OPEN_POS);

        // Step 9: Go Forward
        robot.setDrivePower(0.4,0.4);
        sleep(1000);
        robot.setDrivePower(0.0,0.0);

        // Step 9: Back up
        robot.setDrivePower(-0.4,-0.4);
        sleep(500);
        robot.setDrivePower(0.0,0.0);
    }
}
