package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_MAX_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_MIN_POS;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@TeleOp(name="CommonFunctions", group="Stuff")
@Disabled
public class Auto_CommonFunctions extends LinearOpMode {

    /* Declare OpMode members. */
    protected ElapsedTime runtime = new ElapsedTime();
    CrushyHardware robot = new CrushyHardware();

    @Override
    public void runOpMode() {
    }

    public void forwardDriveInches(int inches) {
        int targetTicks = 0;

        // Ticks per revolution is 1220
        targetTicks = 120 * inches;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for encoders reset
        while (robot.leftFront.getCurrentPosition() != 0){}

        robotDrive(0.4);

        while (robot.leftFront.getCurrentPosition() < targetTicks) {}

        robot.setDrivePower(0.0,0.0,0.0,0.0);
        sleep(500);

    }

    public void backwardDriveInches(int inches) {
        int targetTicks = 0;

        // Ticks per revolution is 1220
        targetTicks = -120 * inches;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for encoders reset
        while (robot.leftFront.getCurrentPosition() != 0){
            telemetry.addData("Status", "Stuck in Get Current Position");
            telemetry.update();

        }

        robotDrive(-0.4);

        while (robot.leftFront.getCurrentPosition() > targetTicks) {
            telemetry.addData("Status", "Stuck in greater than target ticks");
            telemetry.update();
        }

        robot.setDrivePower(0.0,0.0,0.0,0.0);
        sleep(500);
    }

    public void robotStop() {

        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
        sleep(500);
    }

    public void robotDrive(double power) {
        robot.setDrivePower(power, power, power, power);
    }

    public void robotStrafe(double power, String direction) {
        if (direction == "left") {
            robot.setDrivePower(-power, power, power, -power);
        }
        else {
            robot.setDrivePower(power, -power, -power, power);
        }
    }

    public void robotRotate(double power, String direction) {
        if (direction == "left") {
            robot.setDrivePower(-power, power, -power, power);
        }
        else {
            robot.setDrivePower(power, -power, power, -power);
        }
    }

    public void raiseJewelArm() {

        robot.jewelArmServo.setPosition(JEWEL_ARM_SERVO_MAX_POS);
        sleep(1000);
    }

    public void lowerJewelArm() {

        robot.jewelArmServo.setPosition(JEWEL_ARM_SERVO_MIN_POS);
        sleep(1000);
    }

    public void closeGlyphGrabber() {
        robot.rightGlyphServo.setPosition(robot.RIGHT_GLYPH_SERVO_CLOSED_POS);
        robot.leftGlyphServo.setPosition(robot.LEFT_GLYPH_SERVO_CLOSED_POS);
        sleep(400);
    }

    public void openGlyphGrabber() {
        robot.rightGlyphServo.setPosition(robot.RIGHT_GLYPH_SERVO_OPEN_POS);
        robot.leftGlyphServo.setPosition(robot.LEFT_GLYPH_SERVO_OPEN_POS);
        sleep(400);
    }


    public void raiseGlyph() {
        robot.upperCenterLift.setPower(-0.4);
        robot.lowerCenterLift.setPower(-0.4);
        sleep(500);
        robot.upperCenterLift.setPower(0.0);
        robot.lowerCenterLift.setPower(0.0);
    }

    public void lowerGlyph() {
        robot.upperCenterLift.setPower(0.4);
        robot.lowerCenterLift.setPower(0.4);
        sleep(500);
        robot.upperCenterLift.setPower(0.0);
        robot.lowerCenterLift.setPower(0.0);
    }

    public Boolean isRed(ColorSensor localColorSensor) {

        float hsv[] = {0F,0F,0F};

        Color.RGBToHSV(localColorSensor.red() * 8, localColorSensor.green() * 8, localColorSensor.blue() * 8, hsv);

        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) {
            return true;
        } else {
            return false;
        }
    }

    public Boolean isWhiteLine(ColorSensor localColorSensor) {

        float hsv[] = {0F,0F,0F};

        localColorSensor.enableLed(true);

        if ((localColorSensor.red() > 180) && (localColorSensor.green() > 180) && (localColorSensor.blue() > 180)){
            return true;
        } else {
            return false;
        }

    }

    public boolean isBlue(ColorSensor localColorSensor) {

        float hsv[] = {0F,0F,0F};

        localColorSensor.enableLed(false);

        Color.RGBToHSV(localColorSensor.red() * 8, localColorSensor.green() * 8, localColorSensor.blue() * 8, hsv);


        if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) {
            return true;
        } else {
            return false;
        }
    }

    public void turnRight(int degrees){

        Orientation angles;

        angles = robot.gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        heading = convertHeading(heading);

        telemetry.addData("Heading:", heading);

        double targetHeading = heading - degrees;
        targetHeading = convertHeading(targetHeading);

        telemetry.addData("Target:", targetHeading);
        telemetry.update();

        sleep (500);

        while(Math.abs(heading - targetHeading) > 1) {
            robotRotate(0.4,"right");

            angles = robot.gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            heading = convertHeading(heading);

            telemetry.addData("Heading:", heading);
            telemetry.addData("Target:", targetHeading);
            telemetry.update();
        }

        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
    }

    public void turnLeft(int degrees){

        Orientation angles;

        angles = robot.gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        heading = convertHeading(heading);

        telemetry.addData("Heading:", heading);

        double targetHeading = heading + degrees;
        targetHeading = convertHeading(targetHeading);

        telemetry.addData("Target:", targetHeading);
        telemetry.update();

        sleep (500);

        while(Math.abs(heading - targetHeading) > 1) {
            robotRotate(0.4,"left");

            angles = robot.gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            heading = convertHeading(heading);

            telemetry.addData("Heading:", heading);
            telemetry.addData("Target:", targetHeading);
            telemetry.update();
        }

        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
    }

    double convertHeading (double heading){
        if (heading < 0){
            return 360 + heading;
        }

        if (heading > 360) {
            return heading - 360;
        }

        return heading;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void activateVuMark() {
        robot.relicTrackables.activate();
        //sleep(1000);
    }

    public void deactivateVuMark() {
        robot.relicTrackables.deactivate();
        //sleep(500);
    }

    public void readVuMark() {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        robot.vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
        if (robot.vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", robot.vuMark);
        } else {
            telemetry.addData("VuMark", "not visible");
        }
    }
}