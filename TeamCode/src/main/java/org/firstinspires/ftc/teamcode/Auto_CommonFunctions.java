package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_LEFT_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_MIDDLE_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_RIGHT_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_UP_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.JEWEL_ARM_SERVO_DOWN_POS;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@Autonomous(name="Common Functions", group="Autonomous")
@Disabled
public class Auto_CommonFunctions extends LinearOpMode {

    /* Declare OpMode members. */
    protected ElapsedTime runtime = new ElapsedTime();
    CrushyHardware robot = new CrushyHardware();

    static final Integer ALLIANCE_BLUE = 1;
    static final Integer ALLIANCE_RED = 2;

    @Override
    public void runOpMode() {
    }

    public void knockJewel(int alliance) {

        moveServo(robot.jewelArmUpDownServo,JEWEL_ARM_SERVO_UP_POS);
        telemetry.addData("Servo is ", "UP");
        telemetry.update();
        moveServo(robot.jewelArmLeftRightServo,JEWEL_ARM_SERVO_MIDDLE_POS);
        telemetry.addData("Servo is ", "MIDDLE");
        telemetry.update();
        moveServo(robot.jewelArmUpDownServo,JEWEL_ARM_SERVO_DOWN_POS);
        telemetry.addData("Servo is ", "DOWN");
        telemetry.update();

        if (((alliance == ALLIANCE_BLUE) && (isBlue(robot.jewelFarSensor))) ||
                ((alliance == ALLIANCE_RED) && (isRed(robot.jewelFarSensor))) ||
                ((alliance == ALLIANCE_BLUE) && (isRed(robot.jewelNearSensor))) ||
                ((alliance == ALLIANCE_RED) && (isBlue(robot.jewelNearSensor)))) {

            //Swipe left
            moveServo(robot.jewelArmLeftRightServo,JEWEL_ARM_SERVO_LEFT_POS);
            telemetry.addData("Servo is ", "LEFT");
            telemetry.update();
        }
        else if (((alliance == ALLIANCE_BLUE) && (isRed(robot.jewelFarSensor))) ||
                ((alliance == ALLIANCE_RED) && (isBlue(robot.jewelFarSensor))) ||
                ((alliance == ALLIANCE_BLUE) && (isBlue(robot.jewelFarSensor))) ||
                ((alliance == ALLIANCE_RED) && (isRed(robot.jewelFarSensor)))
                ) {
            //Swipe right
            moveServo(robot.jewelArmLeftRightServo,JEWEL_ARM_SERVO_RIGHT_POS);
            telemetry.addData("Servo is ", "RIGHT");
            telemetry.update();
        }

        moveServo(robot.jewelArmUpDownServo,JEWEL_ARM_SERVO_UP_POS);
        telemetry.addData("Servo is ", "UP");
        telemetry.update();

        moveServo(robot.jewelArmLeftRightServo,JEWEL_ARM_SERVO_MIDDLE_POS);
        telemetry.addData("Servo is ", "MIDDLE");
        telemetry.update();
    }

    public void backwardDriveInches(int inches) {
        int targetTicks = 0;

        // Ticks per revolution is 1220
        targetTicks = 120 * inches;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for encoders reset
        while (opModeIsActive() && (robot.leftFront.getCurrentPosition() != 0)){
            telemetry.addData("Status", "Stuck in Get Current Position");
            telemetry.update();
        }

        startRobot(0.4);

        while (opModeIsActive() && (robot.leftFront.getCurrentPosition() < targetTicks)) {
            telemetry.addData("Status", "Stuck in less than target ticks");
            telemetry.update();
        }

        robot.setDrivePower(0.0,0.0,0.0,0.0);
        sleep(500);

    }

    public void forwardDriveInches(int inches) {
        int targetTicks = 0;

        // Ticks per revolution is 1220
        targetTicks = -120 * inches;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for encoders reset
        while (opModeIsActive() && (robot.leftFront.getCurrentPosition() != 0)){
            telemetry.addData("Status", "Stuck in Get Current Position");
            telemetry.update();
        }

        startRobot(-0.4);

        while (opModeIsActive() && (robot.leftFront.getCurrentPosition() > targetTicks)) {
            telemetry.addData("Status", "Stuck in greater than target ticks");
            telemetry.update();
        }

        robot.setDrivePower(0.0,0.0,0.0,0.0);
        sleep(500);
    }

    public void stopRobot() {
        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
        sleep(500);
    }

    public void startRobot(double power) {
        robot.setDrivePower(power, power, power, power);
    }

    public void strafeRobot(double power, String direction) {
        if (direction == "left") {
            robot.setDrivePower(-power, power, power, -power);
        }
        else {
            robot.setDrivePower(power, -power, -power, power);
        }
    }

    public void rotateRobot(double power, String direction) {
        if (direction == "left") {
            robot.setDrivePower(-power, power, -power, power);
        }
        else {
            robot.setDrivePower(power, -power, power, -power);
        }
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

        while(opModeIsActive() && (Math.abs(heading - targetHeading) > 1)) {
            rotateRobot(0.4,"right");

            angles = robot.gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            heading = convertHeading(heading);

            telemetry.addData("Heading:", heading);
            telemetry.addData("Target:", targetHeading);
            telemetry.update();
        }
        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
        sleep(250);
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

        while(opModeIsActive() && (Math.abs(heading - targetHeading) > 1)) {
            rotateRobot(0.4,"left");

            angles = robot.gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            heading = convertHeading(heading);

            telemetry.addData("Heading:", heading);
            telemetry.addData("Target:", targetHeading);
            telemetry.update();
        }
        telemetry.addData("Exit", "While");
        telemetry.update();

        robot.setDrivePower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData("Exit", "TurnLeft");
        telemetry.update();
        sleep(250);
    }

    public void moveServo(Servo localServo, double targetPosition){

        double currentPosition = localServo.getPosition();

        while (opModeIsActive() && (currentPosition != targetPosition)) {

            if (currentPosition < targetPosition) {
                currentPosition += 0.01;

                if (currentPosition > targetPosition)
                {
                    currentPosition = targetPosition;
                }
            } else {
                currentPosition -= 0.01;

                if (currentPosition < targetPosition)
                {
                    currentPosition = targetPosition;
                }
            }

            localServo.setPosition(currentPosition);
            sleep(30);
        }
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

    public void readPictograph() {

        long StartTime = ElapsedTime.MILLIS_IN_NANO;

        robot.relicTrackables.activate();
        telemetry.addData("Activate VuMark", "Activate VuMark");
        telemetry.update();

        while (opModeIsActive() && !((robot.pictograph != RelicRecoveryVuMark.UNKNOWN) || ((ElapsedTime.MILLIS_IN_NANO - StartTime) < 5000))) {

            /**
             * See if any of the instances of relicTemplate are currently visible.
             * RelicRecoveryVuMark is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT.
             */
            robot.pictograph = RelicRecoveryVuMark.from(robot.relicTemplate);
            telemetry.addData("I see", robot.pictograph);
            telemetry.update();
        }

        robot.relicTrackables.deactivate();
        telemetry.addData("Deactivate VuMark", "Deactivate VuMark");
        telemetry.update();
    }
}