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

        //
        //  Test that the Left Back motor is set up correctly
        //
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && (robot.leftBack.getCurrentPosition() != 0)) {
            telemetry.addData("Left Back", "Waiting on encoder");
            telemetry.update();
        }
        telemetry.addData("Left Back", "Running");
        telemetry.update();
        robot.setDrivePower(0.0, 0.0, 0.5, 0.0);
        sleep(5000);
        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData("Left Back Encoder Value", robot.leftBack.getCurrentPosition());
        telemetry.update();
        sleep(5000);

        //
        //  Test that the Right Front motor is set up correctly
        //
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && (robot.rightFront.getCurrentPosition() != 0)) {
            telemetry.addData("Right Front", "Waiting on encoder");
            telemetry.update();
        }
        telemetry.addData("Right Front", "Running");
        telemetry.update();
        robot.setDrivePower(0.0, 0.5, 0.0, 0.0);
        sleep(5000);
        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData("Right Front Encoder Value", robot.rightFront.getCurrentPosition());
        telemetry.update();
        sleep(5000);

        //
        //  Test that the Right Back motor is set up correctly
        //
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive() && (robot.rightBack.getCurrentPosition() != 0)) {
            telemetry.addData("Right Back", "Waiting on encoder");
            telemetry.update();
        }
        telemetry.addData("Right Back", "Running");
        telemetry.update();
        robot.setDrivePower(0.0, 0.0, 0.0, 0.5);
        sleep(5000);
        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData("Right Back Encoder Value", robot.rightBack.getCurrentPosition());
        telemetry.update();
        sleep(5000);
    }
}