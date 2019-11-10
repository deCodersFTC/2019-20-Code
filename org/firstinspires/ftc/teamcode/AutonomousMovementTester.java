package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.lang.Math;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "AutonomousMovementTester" ,group = "Testers")

public class AutonomousMovementTester extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AeAnL1T/////AAABmTZwSvxDH0lVnljgy5pBgO8UdOWEaAiKyXqKqbABovZRTXBALSqlE0OHSRjJfjhCNHaOesi3e47zVd9aP/HvRyXToIZJvvzHxcjiDn4oYfAonwBGJdtJ2tbMZr91LDtH+xg3m1jiyA+UqCx0X9y0aKuKm4elTo5W9gayS0gW1T7bi1ww30yNScGGQABuYzqth/aUB50JDBnjpJV1Um1RljzpYq9RYQzOe1e+UTdCbznQSwEhFxsQmEM40oi6jjyUIWN+Ud3zcYvJ9hM2vbDOab/a1QZBEF6X/T/NCkeCsjQFj7yhA+hLFRMakOikLOb9ZDkrau7g+AxJ7M94puQiJzkkv6VTYn/C64jhur7CxzfP";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    public DcMotor  fl;
    public DcMotor  fr;
    public DcMotor  bl;
    public DcMotor  br;
    public DistanceSensor heightSensor;

    static final double     COUNTS_PER_MOTOR_REV    = 2240;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1/40;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.1;
    static final double     TURN_SPEED              = 0.1;
    static final double     SLIDE_SPEED             = 0.4;
    static final double     DRIVE_BASE_DIAMETER     = 18.2;
    double origAngle;
    Orientation turnAngles;
    BNO055IMU imu;
    double targetAngle;
    double difference;

    ElapsedTime runtime = new ElapsedTime();


    public void encoderDrive (double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = fl.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = fr.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = bl.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = br.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            fl.setTargetPosition(newLeftFrontTarget);
            fr.setTargetPosition(newRightFrontTarget);
            bl.setTargetPosition(newLeftBackTarget);
            br.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //bl.setPower(Math.abs(speed));
            //fl.setPower(Math.abs(speed));
            //br.setPower(Math.abs(speed));
            fr.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && ( bl.isBusy() &&  br.isBusy() && fl.isBusy() && fr.isBusy())) {


            }

            // Stop all motion;
            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            //Set to RUN_USING_ENCODER
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void Backwards(double distance){
        distance = (distance * 0.95) / 2;
        encoderDrive(DRIVE_SPEED,-distance,-distance,distance,distance, 1);
    }
    public void Forwards(double distance){
      distance = (distance * 0.95) / 2;
        encoderDrive(DRIVE_SPEED, distance, distance, -distance, -distance, 1);
    }
    public void TurnLeft(double degrees){
        degrees*=(DRIVE_BASE_DIAMETER * 3.14)/360;
        encoderDrive(TURN_SPEED, -degrees, -degrees, -degrees,-degrees,1);
    }
    public void TurnRight(double degrees){
        degrees*=(DRIVE_BASE_DIAMETER * 3.14)/360;
      encoderDrive(TURN_SPEED, degrees, degrees, degrees, degrees, 1);
    }
    public void slideRight(double distance){
      distance = (distance * 0.95) / 2;
        encoderDrive(SLIDE_SPEED,-distance,distance,-distance,distance,1);
    }
    public void slideLeft(double distance){
      distance = (distance * 0.95) / 2;
        encoderDrive(SLIDE_SPEED,distance,-distance,distance,-distance,1);
    }
    public void stopAllMotors(){
      encoderDrive(0,0,0,0,0,0.5);
    }
    public void SAM(){
      stopAllMotors();
    }

    public void AccurateTurn(double degrees){
        Orientation turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double origAngle = turnAngles.firstAngle;
        double targetAngle = origAngle + degrees;
        double difference = degrees;
        while (Math.abs(difference) > 1) {
            TurnRight(difference * 0.9);
            turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            difference = targetAngle - turnAngles.firstAngle;
            telemetry.addData("Difference", difference);
            telemetry.addData("Target angle", targetAngle);
            telemetry.addData("Current angle", turnAngles.firstAngle);
            telemetry.update();
            sleep(5000);

        }


    }

    public void runOpMode(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr  = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br  = hardwareMap.get(DcMotor.class, "br" );

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            //AccurateTurn(360);
            encoderDrive(TURN_SPEED, 12.5, 12.5, 12.5, 12.5, 1);
            SAM();
            sleep(1000);

            /* Tasks that need to be programed:
             * 1. Pull Foundation into building site
             * 2. Bring as many stones from loading zone into building zone
             * 3. Bring skystone from loading to building zone (look at game manual for method of randomizing skystones)
             * 4. Put as many stones in foundation as possible
             * 5. Park on dividing line between Building and Loading zones
             *
             * Look at the map for point break down at: http://decodersftc.com/gamemap.png
             * Consider not using encoders as only source of data but also using sensors on the robot such as distance sensors, etc.
             */



            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                  telemetry.addData("# Object Detected", updatedRecognitions.size());

                  // step through the list of recognitions and display boundary info.
                  int i = 0;
                  for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                      recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                  }
                  telemetry.update();
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
            break;
        }

        }

        private void initVuforia() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.8;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }
}
