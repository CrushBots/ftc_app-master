package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Test Relic Hand Wrist", group="Test")
//@Disabled
public class Test_RelicHandWrist extends Auto_CommonFunctions {

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

        // Test 1: Relic wrist
        telemetry.addData("Testing ", "Relic");
        telemetry.update();
        sleep(2500);

        //moveServo(robot.relicHandServo, robot.RELIC_HAND_SERVO_OPEN_POS);
        telemetry.addData("", "Hand open");
        telemetry.update();
        sleep(2500);

        //moveServo(robot.relicHandServo, robot.RELIC_HAND_SERVO_CLOSE_POS);
        telemetry.addData("", "Hand close");
        telemetry.update();
        sleep(2500);

        //moveServo(robot.relicWristServo, robot.RELIC_WRIST_SERVO_UP_POS);
        telemetry.addData("", "Up position");
        telemetry.update();
        sleep(2500);

        //moveServo(robot.relicWristServo, robot.RELIC_WRIST_SERVO_DOWN_POS);
        telemetry.addData("", "Down position");
        telemetry.update();
        sleep(2500);


    }
}