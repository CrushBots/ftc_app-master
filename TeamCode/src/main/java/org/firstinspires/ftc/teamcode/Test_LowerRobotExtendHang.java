package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2018-2019 FTC season
 */

@Autonomous(name="Test Lower Robot Extend Hang", group="Test")
//@Disabled
public class Test_LowerRobotExtendHang extends Auto_SeasonFunctions {

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

        telemetry.addData("Lower Robot Extend Hang", "Running");
        telemetry.update();
        lowerRobotExtendHang();
        sleep(1000);
   }
}