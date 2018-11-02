package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by CrushBots for the 2018-2019 FTC season
 */

@Autonomous(name="Common Functions", group="Autonomous")
@Disabled
public class Auto_SeasonFunctions extends Auto_CommonFunctions {

    public void lowerRobotExtendHang() {
        robot.hangMotor.setPower(1.0);
        while (opModeIsActive() && (robot.upTouchSensor.getState())){
            telemetry.addData("Status", "Lowering Robot");
            telemetry.addData( "State", robot.upTouchSensor.getState());
            telemetry.update();
        }
        robot.hangMotor.setPower(0.0);
    }

    public void downHangMotor() {
        robot.hangMotor.setPower(-1.0);
        while (opModeIsActive() && (robot.downTouchSensor.getState())){
            telemetry.addData("Status", "Raising Robot");
            telemetry.update();
        }
        robot.hangMotor.setPower(0.0);
    }
}