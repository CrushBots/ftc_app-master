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

    public void rampUpStart () {
        robot.flopPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flopPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.flopPulley.setPower(0.5);
        robot.flopPulleyMovingUp = true;

    }
    public void rampUpEnd () {

        while (!(robot.flopPulley.getCurrentPosition() < 1250)){
            robot.flopPulleyUp = true;
            robot.flopPulley.setPower(0.0);
            robot.flopPulleyMovingUp = false;
        }

    }

    public void rampDownStart () {
        robot.flopPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flopPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.flopPulley.setPower(-0.1);
        robot.flopPulleyMovingDown = true;
    }

    public void rampDownEnd () {

        while (!(robot.flopPulley.getCurrentPosition() > -1300)) {
            robot.flopPulleyUp = false;
            robot.flopPulley.setPower(0.0);
            robot.flopPulleyMovingDown = false;
        }
    }



    public void flopBackStart (){

        robot.flopRamp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flopRamp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.flopRamp.setPower(0.15);
        robot.flopRampMovingBack = true;
    }

    public void flopBackEnd (){

        while (!(robot.flopRamp.getCurrentPosition() < 225)){
            robot.flopRampForward = false;
            robot.flopRamp.setPower(0.0);
            robot.flopRampMovingBack = false;
        }
    }

    public void flopForwardStart(){

        robot.flopRamp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flopRamp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.flopRamp.setPower(-0.3);
        robot.flopRampMovingForward = true;
    }

    public void flopForwardEnd (){

        if (!(robot.flopRamp.getCurrentPosition() > -250)){
            robot.flopRampForward = true;
            robot.flopRamp.setPower(0.0);
            robot.flopRampMovingForward = false;
        }
    }

    public void relicArmOutStart (){

        robot.relicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.relicArm.setPower(1.0);
        robot.relicArmMovingOut = true;
    }

    public void relicArmOutEnd (){

        if (!(robot.relicArm.getCurrentPosition() < 12500)){
            robot.relicArmOut = true;
            robot.relicArm.setPower(0.0);
            robot.relicArmMovingOut = false;
        }
    }

    public void relicArmInStart (){

        robot.relicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.relicArm.setPower(-0.5);
        robot.relicArmMovingIn = true;
    }

    public void relicArmInEnd (){

        if (!(robot.relicArm.getCurrentPosition() > -12500)){
            robot.relicArmOut = false;
            robot.relicArm.setPower(0.0);
            robot.relicArmMovingIn = false;
        }
    }


    public void startMotorUsingEncoder (DcMotor motor, float power){

        if (motor.getPower() == 0.0){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setPower(power);
        }
    }

    public void stopMotorAtPosition (DcMotor motor, int stopPosition){

        boolean stopMotor = false;

        if (stopPosition > 0) {
            if (motor.getCurrentPosition() >= stopPosition){
                stopMotor = true;
            }
        }
        else {
            if (motor.getCurrentPosition() <= stopPosition){
                stopMotor = true;
            }
        }

        if (stopMotor) {
            robot.relicArmOut = false;
            motor.setPower(0.0);
        }
    }
}