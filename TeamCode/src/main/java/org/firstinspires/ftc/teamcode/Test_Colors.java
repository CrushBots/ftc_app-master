package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2018-2019 FTC season
 */

@Autonomous(name="Test Colors", group="Test")
//@Disabled
public class Test_Colors extends Auto_SeasonFunctions {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
        }

        while (opModeIsActive()) {

            float hsv[] = {0F, 0F, 0F};

            robot.leftLightSensor.enableLed(false);

            Color.RGBToHSV(robot.leftLightSensor.red() * 8, robot.leftLightSensor.green() * 8, robot.leftLightSensor.blue() * 8, hsv);
            telemetry.addData("hsv[0] =", hsv[0]);
            telemetry.addData("hsv =[1] =", hsv[1]);

            if (isWhite(robot.leftLightSensor)) {
                telemetry.addData("Color =", "White");
            }
            else {
                if (isYellow(robot.leftLightSensor)) {
                    telemetry.addData("Color =", "Yellow");
                }
                else {
                    telemetry.addData("Color =", "Unknown");
                }
            }

            robot.centerLightSensor.enableLed(false);

            Color.RGBToHSV(robot.centerLightSensor.red() * 8, robot.centerLightSensor.green() * 8, robot.centerLightSensor.blue() * 8, hsv);
            telemetry.addData("hsv[0] =", hsv[0]);
            telemetry.addData("hsv =[1] =", hsv[1]);

            if (isWhite(robot.centerLightSensor)) {
                telemetry.addData("Color =", "White");
            }
            else {
                if (isYellow(robot.centerLightSensor)) {
                    telemetry.addData("Color =", "Yellow");
                }

                else {
                    telemetry.addData("Color =", "Unknown");
                }
            }
            telemetry.update();
        }
    }
}