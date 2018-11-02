package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/* Created by CrushBots for the 2017-2018 FTC season*/

@Autonomous(name="Test DC Motors ", group="Test")
//@Disabled
public class Test_DCMotors extends Auto_SeasonFunctions {

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

        telemetry.addData("Testing ", "Drive motors");
        telemetry.update();
        sleep(2500);

        //
        //  Test that the Left Front motor is set up correctly
        //
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && (robot.leftFront.getCurrentPosition() != 0)) {
            telemetry.addData("Left Front", "Waiting on encoder");
            telemetry.update();
        }
        telemetry.addData("Left Front", "Running");
        telemetry.update();
        robot.setDrivePower(0.5, 0.0, 0.0, 0.0);
        sleep(5000);
        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData("Left Front Encoder Value", robot.leftFront.getCurrentPosition());
        telemetry.update();
        sleep(5000);

    }
}