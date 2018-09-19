package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Thread.sleep;

/**
 * Created by CrushBots for the 2017-2018 FTC season
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

        robot.jewelArmUpDownServo.setPosition(robot.JEWEL_ARM_SERVO_UP_POS);
        robot.jewelArmLeftRightServo.setPosition(robot.JEWEL_ARM_SERVO_MIDDLE_POS);

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
         * FLOP PULLEY - UP
         ******************************************************************/
        if (!robot.flopPulleyUp && gamepad2.right_stick_y < -0.5) {
            startMotorUsingEncoder(robot.flopPulley, 0.5);
            robot.flopRamp.setPower(-0.1);
        }
        if (robot.flopPulley.getPower() > 0.0){
            robot.flopRamp.setPower(-0.1);
        }
        robot.flopPulleyUp = stopPulleyMotor(robot.flopPulleyUp);

        /******************************************************************
         * FLOP PULLEY - DOWN
         ******************************************************************/
        if (robot.flopPulleyUp && gamepad2.right_stick_y > 0.5) {
            startMotorUsingEncoder(robot.flopPulley, -0.1);
        }
        robot.flopPulleyUp = stopMotorAtPosition(robot.flopPulley, -1300, robot.flopPulleyUp);

        /******************************************************************
         * FLOP RAMP - FORWARD
         ******************************************************************/
        if (!robot.flopRampForward && gamepad2.left_stick_y < -0.5) {
            startMotorUsingEncoder(robot.flopRamp, -0.3);
        }
        robot.flopRampForward = stopMotorAtPosition(robot.flopRamp, -250, robot.flopRampForward);

        /******************************************************************
         * FLOP RAMP - BACK
         ******************************************************************/
        if (robot.flopRampForward && gamepad2.left_stick_y > 0.5) {
            startMotorUsingEncoder(robot.flopRamp, 0.15);
        }
        robot.flopRampForward = stopMotorAtPosition(robot.flopRamp, 245, robot.flopRampForward);

        /******************************************************************
         * RELIC ARM - OUT
         ******************************************************************/
        if (!robot.relicArmOut && gamepad2.left_bumper) {
            startMotorUsingEncoder(robot.relicArm, 1.0);
        }
        robot.relicArmOut = stopMotorAtPosition(robot.relicArm, 9000, robot.relicArmOut);

        /******************************************************************
         * RELIC ARM - IN
         ******************************************************************/
        if (robot.relicArmOut && gamepad2.right_bumper) {
            startMotorUsingEncoder(robot.relicArm, -0.5);
        }
        telemetry.addData("RelicArmOut",robot.relicArm.getPower());

        robot.relicArmOut = stopMotorAtPosition(robot.relicArm, -9000, robot.relicArmOut);

        /******************************************************************
         * RELIC WRIST
         ******************************************************************/
        if (gamepad2.dpad_up) {
            robot.relicWristServo.setPosition(robot.RELIC_WRIST_SERVO_UP_POS);
        }
        if (gamepad2.dpad_down) {
            robot.relicWristServo.setPosition(robot.RELIC_WRIST_SERVO_DOWN_POS);
        }

        /******************************************************************
         * RELIC HAND
         ******************************************************************/
        if (gamepad2.dpad_left) {
            robot.relicHandServo.setPosition(robot.RELIC_HAND_SERVO_OPEN_POS);
        }
        if (gamepad2.dpad_right) {
            robot.relicHandServo.setPosition(robot.RELIC_HAND_SERVO_CLOSE_POS);
        }

        /******************************************************************
         * INTAKE
         ******************************************************************/
        if (gamepad2.b) {
            robot.turnOnIntake();
        }
        if (gamepad2.a) {
            robot.turnOffIntake();
        }
        if (gamepad1.b) {
            robot.backwardsIntake();
        }
    }
}