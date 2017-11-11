package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.CrushyHardware.LEFT_GLYPH_SERVO_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.LEFT_GLYPH_SERVO_OPEN_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.RIGHT_GLYPH_SERVO_CLOSED_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.RIGHT_GLYPH_SERVO_OPEN_POS;

/**
 * Created by CrushBots for the 2017-2018 FTC season
 */

@TeleOp(name="Mecanum TeleOp", group="CrushBots")
//@Disabled
public class MecanumTeleOp extends CommonFunctions {
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
    }
}