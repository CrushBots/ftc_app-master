package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@TeleOp(name="Driver TeleOp", group="CrushBots")
//@Disabled
public class DriverTeleOp extends CommonFunctions {

    boolean flopPulleyMoving = false;


    /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    */
    //@Override
    //public void init_loop() {
    //   robot.jewelArmUpDownServo.setPosition(robot.JEWEL_ARM_SERVO_UP_POS);
    //  robot.jewelArmLeftRightServo.setPosition(robot.JEWEL_ARM_SERVO_MIDDLE_POS);
    //}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        double drive;
        double strafe;
        double rotate;
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double speedMultiple = 1.0;

        robot.jewelArmUpDownServo.setPosition(robot.JEWEL_ARM_SERVO_UP_POS);
        robot.jewelArmLeftRightServo.setPosition(robot.JEWEL_ARM_SERVO_MIDDLE_POS);

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
         * FLOP RAMP
         ******************************************************************/
        if (!robot.flopPulleyUp && !robot.flopRampUp && gamepad2.b) {
            upAndFlop();
        }
        if (robot.flopPulleyUp && robot.flopRampUp && gamepad2.a) {
            flopAndDown();
        }

        telemetry.addData("FlopRampUp: ", robot.flopRampUp);
        telemetry.addData("Left Stick Y: ", gamepad2.left_stick_y);
        telemetry.update();

        if (!robot.flopRampUp && gamepad2.left_stick_y < -0.5) {
            robot.flopForward();
        }

        if (robot.flopRampUp && gamepad2.left_stick_y > 0.5) {
            robot.flopBack();
        }

        /******************************************************************
         * RELIC
         ******************************************************************/
        if (gamepad2.dpad_up) {
            robot.relicWristServo.setPosition(robot.RELIC_WRIST_SERVO_UP_POS);
        }
        if (gamepad2.dpad_down) {
            robot.relicWristServo.setPosition(robot.RELIC_WRIST_SERVO_DOWN_POS);
        }
        if (gamepad2.dpad_left) {
            robot.relicHandServo.setPosition(robot.RELIC_HAND_SERVO_OPEN_POS);
        }
        if (gamepad2.dpad_right) {
            robot.relicHandServo.setPosition(robot.RELIC_HAND_SERVO_CLOSE_POS);
        }
        if (gamepad2.left_bumper) {
            robot.relicArm.setPower(0.5);
        } else {
            robot.relicArm.setPower(0.0);
        }
        if (gamepad2.right_bumper) {
            robot.relicArm.setPower(-0.3);
        } else {
            robot.relicArm.setPower(0.0);
        }

        /******************************************************************
         * INTAKE
         ******************************************************************/
        if (gamepad2.y) {
            robot.turnOnIntake();
        }
        if (gamepad2.x) {
            robot.turnOffIntake();
        }
    }
}

