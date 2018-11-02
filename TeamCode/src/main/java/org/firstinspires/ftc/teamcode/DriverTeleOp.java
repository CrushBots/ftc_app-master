package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.CrushyHardware.SAMPLE_ARM_UP_POS;

/**
 * Created by CrushBots for the 2018-2019 FTC season
 */

@TeleOp(name="Driver TeleOp", group="CrushBots")
//@Disabled
public class DriverTeleOp extends CommonFunctions {

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        robot.sampleArmServo.setPosition(SAMPLE_ARM_UP_POS);

        double drive;
        double strafe;
        double rotate;
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double speedMultiple = 1.0;

        /******************************************************************
         * Figure out if the speed is whole or half
         ******************************************************************/
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            speedMultiple = 0.5;
        } else {
            speedMultiple = 1.0;
        }

        /******************************************************************
         * Driver - Left Joy Stick - Forward / Reverse
         * Driver - Right Joy Stick - Left / Right
         ******************************************************************/
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // calculate the base power of each motor based on x1 and y1
        leftFrontPower = speedMultiple * (drive + strafe + rotate);
        rightFrontPower = speedMultiple * (drive - strafe - rotate);
        leftBackPower = speedMultiple * (drive - strafe + rotate);
        rightBackPower = speedMultiple * (drive + strafe - rotate);
        robot.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        /******************************************************************
         * LOWER ROBOT EXTEND HANG
         ******************************************************************/
        if (!robot.upTouchSensor.getState() && gamepad2.right_stick_y < -0.5) {
            robot.hangMotor.setPower(1.0);
        } else {
            robot.hangMotor.setPower(0.0);
        }

        /******************************************************************
         * RAISE ROBOT SHRINK HANG
         ******************************************************************/
        if (!robot.downTouchSensor.getState() && gamepad2.right_stick_y > 0.5) {
            robot.hangMotor.setPower(-1.0);
        } else {
            robot.hangMotor.setPower(0.0);
        }
    }
}