 package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Thread.sleep;

 /**
  * Created by CrushBots for the 2017-2018 FTC season
  */

 public class CrushyHardware
{
    /* Public members. */
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor upperCenterLift = null;
    public DcMotor lowerCenterLift = null;
    public DcMotor relicArm = null;
    public Servo rightGlyphServo = null;
    public Servo leftGlyphServo = null;
    public Servo relicWristServo = null;
    public Servo relicFingersServo = null;
    public Servo jewelArmServo = null;
    public ColorSensor sensorColor;
    public BNO055IMU gyro = null;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia = null;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;
    RelicRecoveryVuMark vuMark = null;

    static final double JEWEL_ARM_SERVO_MAX_POS = 1.0;
    static final double JEWEL_ARM_SERVO_MIN_POS = 0.3;

    static final double RIGHT_GLYPH_SERVO_OPEN_POS = 0.2;
    static final double RIGHT_GLYPH_SERVO_CLOSED_POS = 0.5;

    static final double LEFT_GLYPH_SERVO_OPEN_POS = 0.5;
    static final double LEFT_GLYPH_SERVO_CLOSED_POS = 0.3;

    static final double RELIC_WRIST_SERVO_UP_POS = 1.0;
    static final double RELIC_WRIST_SERVO_DOWN_POS = 0.0;

    static final double RELIC_FINGERS_SERVO_UP_POS = 1.0;
    static final double RELIC_FINGERS_SERVO_DOWN_POS = 0.0;


    /* Local members. */
    HardwareMap hwMap = null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public CrushyHardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
         hwMap = ahwMap;

        /**
         *  Define and Initialize Motors
         */
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");
        upperCenterLift = hwMap.dcMotor.get("upperCenterLift");
        lowerCenterLift = hwMap.dcMotor.get("lowerCenterLift");
        relicArm = hwMap.dcMotor.get("relicArm");

        // Set the direction of the motors to FORWARD or REVERSE
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD );
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        upperCenterLift.setDirection(DcMotor.Direction.FORWARD);
        lowerCenterLift.setDirection(DcMotor.Direction.FORWARD);
        relicArm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        upperCenterLift.setPower(0);
        lowerCenterLift.setPower(0);
        relicArm.setPower(0);

        // Set all motors to run with or without encoders.
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperCenterLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerCenterLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run with or without encoders.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperCenterLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerCenterLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all Zero Power Behavior - FLOAT or BREAK
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upperCenterLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerCenterLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /**
         *  Define and set start position on Servos
         */
        jewelArmServo = hwMap.servo.get("jewelArm");
        jewelArmServo.setPosition(JEWEL_ARM_SERVO_MAX_POS);
        rightGlyphServo = hwMap.servo.get("rightGlyph");
        //rightGlyphServo.setPosition(RIGHT_GLYPH_SERVO_CLOSED_POS);
        leftGlyphServo = hwMap.servo.get("leftGlyph");
        //leftGlyphServo.setPosition(LEFT_GLYPH_SERVO_CLOSED_POS);
        //relicWristServo = hwMap.servo.get("relicWrist");
        //relicWristServo.setPosition(RELIC_WRIST_SERVO_DOWN_POS);
        //relicFingersServo = hwMap.servo.get("relicFingers");
        //relicFingersServo.setPosition(RELIC_FINGERS_SERVO_DOWN_POS);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "gyro";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hwMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        /**
         *  Define and setup Color sensors
         */
        //leftBeaconColorSensor = hwMap.colorSensor.get("leftBeacon");
        //leftBeaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        sensorColor = hwMap.get(ColorSensor.class, "colorDistanceSensor");


        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        VuforiaLocalizer.Parameters vuParameters = new VuforiaLocalizer.Parameters();
        vuParameters.vuforiaLicenseKey = "AXGsSd3/////AAAAGfr/fIb4S0kzvQ/FHKNWbEEmqBZLr+0MMwryEhTuYhb0Cvi89bYECD1QnBBec+cclaRl8HOmg0RC24qs1R58Ol3LW7fx8J5Jj9pOUmxMDMowgkeFG7Swg2J1NUCXOqmNPyHw79dGgTJ8kCcagfBc/nZoVKoLR8CFP7vf3uP3rH10GLp1R4qZpS6qJEpqJragorOkEmu7CvkNt2Y5KKIY+NfD5W8BFvdQg34jheSu+WwFUNDz2N46GXFerpon+dgyIqZydvmO79rHtkFhH29ip569TRnPLNz/cEJNAdn6d3JQmUoB7p8uCYmBmmfC1WkJxKrP9NdnVc+VME5jaw62NZy5mXtkfvi+gZF7unlDoVXp";
        vuParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;   // Which camera that we wish to use.
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuParameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }

    public void setDrivePower (double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower){
        double max = 1.0;

        // Normalize the values so neither exceed +/- 1.0
        if (Math.abs(leftFrontPower) > max) {
            max = Math.abs(leftFrontPower);
        }

        if (Math.abs(rightFrontPower) > max) {
            max = Math.abs(rightFrontPower);
        }

        if (Math.abs(leftBackPower) > max) {
            max = Math.abs(leftBackPower);
        }

        if (Math.abs(rightBackPower) > max) {
            max = Math.abs(rightBackPower);
        }

        // Set the power of each motor. Divide by max to avoid invalid values.
        leftFront.setPower(leftFrontPower / max);
        rightFront.setPower(rightFrontPower / max);
        leftBack.setPower(leftBackPower / max);
        rightBack.setPower(rightBackPower / max);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     //* @param periodMs  Length of wait cycle in mSec.
     */
    //public void waitForTick(long periodMs) {

        //long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        //if (remaining > 0) {
            //try {
                //sleep(remaining);
            //} catch (InterruptedException e) {
                //Thread.currentThread().interrupt();
            //}
        //}

        // Reset the cycle clock for the next pass.
        //period.reset();
    //}
}