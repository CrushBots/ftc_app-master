package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Test Servo Positions", group="Test")
//@Disabled
public class Test_ServoPositions extends Auto_CommonFunctions {

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
        moveServo(robot.jewelArmUpDownServo, robot.JEWEL_ARM_SERVO_DOWN_POS);

        // Test 1: Jewel Arm - Up / Down
        telemetry.addData("Testing ", "Left / Right Servo");
        telemetry.update();
        sleep(2500);

        moveServo(robot.jewelArmLeftRightServo, robot.JEWEL_ARM_SERVO_RIGHT_POS);
        telemetry.addData("", "RIGHT Position");
        telemetry.update();
        sleep(2500);

        moveServo(robot.jewelArmLeftRightServo, robot.JEWEL_ARM_SERVO_MIDDLE_POS);
        telemetry.addData("", "MIDDLE Position");
        telemetry.update();
        sleep(2500);

        moveServo(robot.jewelArmLeftRightServo, robot.JEWEL_ARM_SERVO_LEFT_POS);
        telemetry.addData("", "LEFT Position");
        telemetry.update();
        sleep(2500);

    }
}