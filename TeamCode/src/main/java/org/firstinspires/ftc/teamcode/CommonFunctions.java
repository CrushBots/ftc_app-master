package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@TeleOp(name="CommonFunctions", group="Stuff")
@Disabled
public class CommonFunctions extends OpMode {

    /* Declare OpMode members. */
    protected ElapsedTime runtime = new ElapsedTime();
    CrushyHardware robot = new CrushyHardware();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        robot.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    public void startMotorUsingEncoder (DcMotor motor, double power){

        if (motor.getPower() == 0.0){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setPower(power);
        }
    }

    public boolean stopMotorAtPosition (DcMotor motor, int stopPosition, boolean positionBoolean) {

        boolean stopMotor = false;

        if (motor.getPower() > 0.0) {
            if (stopPosition > 0) {
                if (motor.getCurrentPosition() >= stopPosition) {
                    stopMotor = true;
                }
            }
        }
        if (motor.getPower() < 0.0) {
            if (stopPosition < 0) {
                if (motor.getCurrentPosition() <= stopPosition) {
                    stopMotor = true;
                }
            }
        }

        if (stopMotor) {
                positionBoolean = !(positionBoolean);
                motor.setPower(0.0);
        }

        return positionBoolean;
    }

    public boolean stopPulleyMotor (boolean positionBoolean) {

        boolean stopMotor = false;

        if (robot.flopPulley.getPower() > 0.0) {
            if (1250 > 0) {
                if (robot.flopPulley.getCurrentPosition() >= 1250) {
                    stopMotor = true;
                }
            }
        }
        if (robot.flopPulley.getPower() < 0.0) {
            if (1250 < 0) {
                if (robot.flopPulley.getCurrentPosition() <= 1250) {
                    stopMotor = true;
                }
            }
        }

        if (stopMotor) {
            positionBoolean = !(positionBoolean);
            robot.flopPulley.setPower(0.0);
            robot.flopRamp.setPower(0.0);
        }

        return positionBoolean;

    }

    public void upAndFlop () {

        robot.flopPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flopPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flopRamp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flopRamp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.flopPulley.setPower(0.3);
        robot.flopRamp.setPower(-0.1);

        while (robot.flopPulley.getCurrentPosition() < 1250){
            telemetry.addData("Current Position", robot.flopPulley.getCurrentPosition());
            telemetry.update();
        }

        robot.flopPulleyUp = true;
        robot.flopPulley.setPower(0.0);

        robot.flopRamp.setPower(0.0);
        robot.flopForward();
    }

    public void flopAndDown () {

        robot.flopBack();

        robot.flopPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flopPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flopPulley.setPower(-0.1);

        while (robot.flopPulley.getCurrentPosition() > -1300){
        }

        robot.flopPulleyUp = false;
        robot.flopPulley.setPower(0.0);

    }

}