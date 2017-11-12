package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.CrushyHardware.RIGHT_GLYPH_SERVO_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.RIGHT_GLYPH_SERVO_OPEN_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.LEFT_GLYPH_SERVO_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.LEFT_GLYPH_SERVO_OPEN_POS;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@TeleOp(name="Driver TeleOp", group="CrushBots")
//@Disabled
public class DriverTeleOp extends CommonFunctions {

    /* Declare members. */
    private double speedControl = 1.0;

    private static final double shooterChangePower = 0.01;
    private double shooterPower = 0.0;
    private boolean shooterStarted = false;
    private boolean rightBumperIsPressed = false;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        double left;
        double right;
        double max;
        double leftPower;
        double rightPower;

        /*
         * Driver - Left Bumper - Increase Speed
         */
        if (gamepad1.left_bumper) {
            speedControl = 1.0;
        } else {
            speedControl = 1.0;
            telemetry.addData("speedControl", speedControl);
        }

        /*
         * Driver - Right Bumper - Decrease Speed
         */
        if (gamepad1.right_bumper) {
            speedControl = 1.0;
        } else {
            speedControl = 1.0;
        }

        double drive;
        double strafe;
        double rotate;
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        /*
         * Driver - Left Joy Stick - Forward / Reverse
         * Driver - Right Joy Stick - Left / Right
         */
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // calculate the base power of each motor based on x1 and y1
        leftFrontPower = drive + strafe + rotate;
        rightFrontPower = drive - strafe - rotate;
        leftBackPower = drive - strafe + rotate;
        rightBackPower = drive + strafe - rotate;

        robot.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        /*
         * Co-Driver - Right Joy Stick - Center Lift
         */
        if (gamepad2.right_stick_y > 0.1)
        {
            robot.upperCenterLift.setPower(0.1);
            robot.lowerCenterLift.setPower(0.1);
        }
        else if (gamepad2.right_stick_y < -0.1)
        {
            robot.upperCenterLift.setPower(-0.1);
            robot.lowerCenterLift.setPower(-0.1);
        }
        else
        {
            robot.upperCenterLift.setPower(0.0);
            robot.lowerCenterLift.setPower(0.0);
        }


        /*
         * Co-Driver - A Button - Relic Arm Out
         */
        if (gamepad2.a)
        {
            robot.relicArm.setPower(1.0);
        }
        else {
            robot.relicArm.setPower(0.0);
        }

        /*
         * Co-Driver - B Button - Relic Arm In
         */
        if (gamepad2.b)
        {
            robot.relicArm.setPower(-1.0);
        }
        else {
            robot.relicArm.setPower(0.0);
        }


        /*
         * Co-Driver - Left Joy Stick - Glyph Open/Close
         */
        if (gamepad2.left_stick_y > 0.1)
        {
            robot.rightGlyphServo.setPosition(RIGHT_GLYPH_SERVO_OPEN_POS);
            robot.leftGlyphServo.setPosition(LEFT_GLYPH_SERVO_OPEN_POS);

        }
        else if (gamepad2.left_stick_y < -0.1)
        {
            robot.rightGlyphServo.setPosition(RIGHT_GLYPH_SERVO_CLOSED_POS);
            robot.leftGlyphServo.setPosition(LEFT_GLYPH_SERVO_CLOSED_POS);
        }

        Orientation angles;

        angles = robot.gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        telemetry.addData("Heading:", formatAngle(angles.angleUnit, angles.firstAngle));

        if(gamepad1.y){
            double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            heading = convertHeading(heading);

            double targetHeading = heading + 90;

            while(heading < targetHeading) {
                robot.leftBack.setPower(-0.4);
                robot.leftFront.setPower(-0.4);
                robot.rightBack.setPower(0.4);
                robot.rightFront.setPower(0.4);

                angles = robot.gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                heading = convertHeading(heading);
            }

            robot.leftBack.setPower(0.0);
            robot.leftFront.setPower(0.0);
            robot.rightBack.setPower(0.0);
            robot.rightFront.setPower(0.0);
        }
    }

    double convertHeading (double heading){
        if (heading < 0){
            return 360 + heading;
        }

        return heading;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}