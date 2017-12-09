package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Test DC Motors ", group="Test")
//@Disabled
public class Test_DCMotors extends Auto_CommonFunctions {

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

        // Test 1: Drive motors
        telemetry.addData("Testing ", "Drive motors");
        telemetry.update();
        sleep(2500);

        robot.setDrivePower(0.5,0.0,0.0,0.0);
        telemetry.addData("", "Left front drive");
        telemetry.update();
        sleep(2500);

        robot.setDrivePower(0.0,0.5,0.0,0.0);
        telemetry.addData("", "Right front drive");
        telemetry.update();
        sleep(2500);

        robot.setDrivePower(0.0,0.0,0.5,0.0);
        telemetry.addData("", "Left back drive");
        telemetry.update();
        sleep(2500);

        robot.setDrivePower(0.0,0.0,0.0,0.5);
        telemetry.addData("", "Right back drive");
        telemetry.update();
        sleep(2500);
    }
}