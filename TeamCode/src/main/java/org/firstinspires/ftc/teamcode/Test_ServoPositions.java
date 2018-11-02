package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2018-2019 FTC season
 */

@Autonomous(name="Test Servo Positions", group="Test")
//@Disabled
public class Test_ServoPositions extends Auto_SeasonFunctions {

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

        telemetry.addData("Testing ", "Servo Positions");
        telemetry.update();
        sleep(500);

        // Test: Sample Arm
        moveServo(robot.sampleArmServo, robot.SAMPLE_ARM_OUT_POS);
        telemetry.addData("Sample Arm ", " Out Positions");
        telemetry.update();
        sleep(500);

        moveServo(robot.centerServoScanner, robot.CENTER_SCANNER_RIGHT_POS);
        telemetry.addData("Center Scanner ", " Right Positions");
        telemetry.update();
        sleep(500);

        moveServo(robot.centerServoScanner,robot.CENTER_SCANNER_LEFT_POS);
        telemetry.addData("Center Scanner ", " Left Positions");
        telemetry.update();
        sleep(500);

        moveServo(robot.centerServoScanner, robot.CENTER_SCANNER_FRONT_POS);
        telemetry.addData("Center Scanner ", " Front Positions");
        telemetry.update();
        sleep(500);

        moveServo(robot.leftServoScanner, robot.LEFT_SCANNER_RIGHT_POS);
        telemetry.addData("Left Scanner ", " Right Positions");
        telemetry.update();
        sleep(500);

        moveServo(robot.leftServoScanner,robot.LEFT_SCANNER_LEFT_POS);
        telemetry.addData("Left Scanner ", " Left Positions");
        telemetry.update();
        sleep(500);

        moveServo(robot.leftServoScanner,robot.LEFT_SCANNER_FRONT_POS);
        telemetry.addData("Left Scanner ", " Front Positions");
        telemetry.update();
        sleep(500);

        moveServo(robot.sampleArmServo,robot.SAMPLE_ARM_UP_POS);
        telemetry.addData("Sample Arm ", " Up Positions");
        telemetry.update();
        sleep(500);
    }
}