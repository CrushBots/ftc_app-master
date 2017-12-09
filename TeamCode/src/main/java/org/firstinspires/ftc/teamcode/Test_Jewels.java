package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_DOWN_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_LEFT_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_MIDDLE_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_RIGHT_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_UP_POS;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Test Jewels", group="Test")
//@Disabled
public class Test_Jewels extends Auto_CommonFunctions {

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

        // Test 1: Blue Alliance with Red on left and Blue on right
        telemetry.addData("Test 1 Setup", "Red / Blue");
        telemetry.update();
        sleep(5000);

        knockJewel(ALLIANCE_BLUE);

        telemetry.addData("Test 1 Results", "Red knocked off");
        telemetry.update();
        sleep(5000);

        // Test 2: Red Alliance with Red on left and Blue on right
        telemetry.addData("Test 2 Setup", "Red / Blue");
        telemetry.update();
        sleep(5000);

        knockJewel(ALLIANCE_RED);

        telemetry.addData("Test 2 Results", "Blue knocked off");
        telemetry.update();
        sleep(5000);

        // Test 3: Blue Alliance with Blue on left and Red on right
        telemetry.addData("Test 3 Setup", "Blue / Red");
        telemetry.update();
        sleep(5000);

        knockJewel(ALLIANCE_BLUE);

        telemetry.addData("Test 3 Results", "Red knocked off");
        telemetry.update();
        sleep(5000);

        // Test 4: Red Alliance with Blue on left and Red on right
        telemetry.addData("Test 4 Setup", "Blue / Red");
        telemetry.update();
        sleep(5000);

        knockJewel(ALLIANCE_RED);

        telemetry.addData("Test 4 Results", "Blue knocked off");
        telemetry.update();
        sleep(5000);
    }
}