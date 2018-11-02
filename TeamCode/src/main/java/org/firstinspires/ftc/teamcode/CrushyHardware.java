 package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

 /**
  * Created by CrushBots for the 2017-2018 FTC season
  */

 public class CrushyHardware
{
    /* Public members. */
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor hangMotor = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;

    public Servo centerServoScanner = null;
    public Servo leftServoScanner = null;
    public Servo sampleArmServo = null;

    public ColorSensor leftLightSensor = null;
    public ColorSensor centerLightSensor = null;

    public BNO055IMU gyro = null;

    public DigitalChannel downTouchSensor = null;
    public DigitalChannel upTouchSensor = null;

    static final double SAMPLE_ARM_OUT_POS = 0.85;
    static final double SAMPLE_ARM_UP_POS = 0.0;

    static final double CENTER_SCANNER_RIGHT_POS = 0.0;
    static final double CENTER_SCANNER_FRONT_POS = 0.50;
    static final double CENTER_SCANNER_LEFT_POS = 0.70;

    static final double LEFT_SCANNER_RIGHT_POS = 0.0;
    static final double LEFT_SCANNER_FRONT_POS = 0.50;
    static final double LEFT_SCANNER_LEFT_POS = 0.70;



    //public VuforiaTrackables relicTrackables = null;
    //public VuforiaTrackable relicTemplate = null;
    //public RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;

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

        /**********************************************************************************
         *  Define and Initialize Motors
         **********************************************************************************/
        hangMotor = hwMap.dcMotor.get("hangMotor");
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");

        // Set the direction of the motors to FORWARD or REVERSE
        hangMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD );
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        hangMotor.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Set all motors to RUN_USING_ENCODER or RUN_WITHOUT_ENCODER.
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all Zero Power Behavior - FLOAT or BREAK
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /**********************************************************************************
         *  Define Servos
         **********************************************************************************/
        leftServoScanner = hwMap.servo.get("left ServoScanner");
        centerServoScanner = hwMap.servo.get("centerServoScanner");
        sampleArmServo = hwMap.servo.get("sampleArmServo");

        /**********************************************************************************
         *  Define and setup Gyro
         **********************************************************************************/
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

        /**********************************************************************************
         *  Define and setup Color sensors
         **********************************************************************************/
        leftLightSensor = hwMap.get(ColorSensor.class, "leftLigthSensor");
        centerLightSensor = hwMap.get(ColorSensor.class, "centerLightSensor");

        /**********************************************************************************
        *  Define and setup Touch Sensor
        **********************************************************************************/
        upTouchSensor = hwMap.get(DigitalChannel.class, "upTouchSensor");
        upTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        downTouchSensor = hwMap.get(DigitalChannel.class, "downTouchSensor");
        downTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        /**********************************************************************************
         *  Define and setup Vuforia
         **********************************************************************************/
        //VuforiaLocalizer vuforia;
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        //int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters vuforiaPparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        //VuforiaLocalizer.Parameters vuforiaPparameters = new VuforiaLocalizer.Parameters();

        // License key to use Vuforia is from https://developer.vuforia.com/license-manager.
        //vuforiaPparameters.vuforiaLicenseKey = "AXGsSd3/////AAAAGfr/fIb4S0kzvQ/FHKNWbEEmqBZLr+0MMwryEhTuYhb0Cvi89bYECD1QnBBec+cclaRl8HOmg0RC24qs1R58Ol3LW7fx8J5Jj9pOUmxMDMowgkeFG7Swg2J1NUCXOqmNPyHw79dGgTJ8kCcagfBc/nZoVKoLR8CFP7vf3uP3rH10GLp1R4qZpS6qJEpqJragorOkEmu7CvkNt2Y5KKIY+NfD5W8BFvdQg34jheSu+WwFUNDz2N46GXFerpon+dgyIqZydvmO79rHtkFhH29ip569TRnPLNz/cEJNAdn6d3JQmUoB7p8uCYmBmmfC1WkJxKrP9NdnVc+VME5jaw62NZy5mXtkfvi+gZF7unlDoVXp";

        // Indicate which camera to use: BACK (HiRes camera with greater range) or FRONT
        //vuforiaPparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //vuforia = ClassFactory.createVuforiaLocalizer(vuforiaPparameters);

        // Load the data set containing the VuMarks for Relic Recovery.
        //relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        //relicTemplate = relicTrackables.get(0);
        //relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

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


}

