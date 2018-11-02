package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.CrushyHardware.SAMPLE_ARM_OUT_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.SAMPLE_ARM_UP_POS;

/**
 * Created by CrushBots for the 2018-2019 FTC season
 */

@Autonomous(name="Right", group="Autonomous")
//@Disabled
public class Auto_Right extends Auto_SeasonFunctions {

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

        // Step 1: Lower robot to ground
        lowerRobotExtendHang();

        // Step 2: Unhook from lander
        strafeRobot(0.5, "right");
        sleep(100);
        stopRobot();

        // Step 3: Robot drives forward to close to the minerals
        forwardDriveInches(18);
        sleep(500);

        // Step 4: Figure out where the Gold Mineral is
        if (isYellow(robot.centerLightSensor)){
            forwardDriveInches(10);
            backwardDriveInches(11);
            turnRight(80);
            forwardDriveInches(60);
        }
        else {
            moveServo(robot.sampleArmServo,SAMPLE_ARM_OUT_POS);
            sleep(500);
            if (isYellow(robot.leftLightSensor)){
                moveServo(robot.sampleArmServo,SAMPLE_ARM_UP_POS);
                backwardDriveInches(5);
                turnLeft(40);
                forwardDriveInches(20);
                turnLeft(60);
                forwardDriveInches(35);
            }
            else{
                moveServo(robot.sampleArmServo,SAMPLE_ARM_UP_POS);
                backwardDriveInches(6);
                turnRight(40);
                forwardDriveInches(15);
                turnRight(40);
                forwardDriveInches(45);
            }
        }
    }
}
