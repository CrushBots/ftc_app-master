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

        robot.jewelArmUpDownServo.setPosition(robot.JEWEL_ARM_SERVO_UP_POS);
        /******************************************************************
         * Driver - Left Joy Stick - Forward / Reverse
         * Driver - Right Joy Stick - Left / Right
         ******************************************************************/
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // calculate the base power of each motor based on x1 and y1
        leftFrontPower = drive + strafe + rotate;
        rightFrontPower = drive - strafe - rotate;
        leftBackPower = drive - strafe + rotate;
        rightBackPower = drive + strafe - rotate;
        robot.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        /******************************************************************
         * Name? - Left Joy Stick - Relic Wrist Up / Down
         * Name? - Right Joy Stick - Relic Arm Out / In
         * Name? - xxx - Relic Hand Open / Closed
         ******************************************************************/
        if (gamepad2.left_stick_y > 0.0 ) {
            robot.flipRamp.setPower(-0.6);
        }
        else {
            robot.flipRamp.setPower(0.0);
        }

        if (gamepad2.left_stick_y < 0.0 ) {
            robot.flipRamp.setPower(0.3);
        }
        else {
            robot.flipRamp.setPower(0.0);
        }

        if (gamepad2.right_stick_y > 0.0 ) {
            robot.flopForward();
        }

        if (gamepad2.right_stick_y < 0.0 ) {
            robot.flopBack();
        }

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

        if (gamepad2.y) {
            robot.turnOnIntake();
        }

        if (gamepad2.x) {
            robot.turnOffIntake();
        }

        if (gamepad2.a) {
            robot.flopPulley.setPower(0.5);
        }
        else {
            robot.flopPulley.setPower(0.0);
        }

        if (gamepad2.b) {
            robot.flopPulley.setPower(-0.5);
        }
        else {
            robot.flopPulley.setPower(0.0);
        }

        if (gamepad2.left_bumper) {
            if (robot.relicArm.getPower() > 0) {
                robot.relicArm.setPower(0.0);
            } else {
                robot.relicArm.setPower(0.3);
            }
        }

        if (gamepad2.right_bumper) {
            if (robot.relicArm.getPower() > 0) {
                robot.relicArm.setPower(0.0);
            } else {
                robot.relicArm.setPower(-0.3);
            }
        }
    }
}