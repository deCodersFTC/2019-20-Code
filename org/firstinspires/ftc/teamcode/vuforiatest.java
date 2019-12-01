package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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

@Autonomous(name="Vuforia Test", group="Pushbot")
public class vuforiatest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private DcMotor br;
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor foundationMotor;
    private DcMotor extend;
    private CRServo grab;

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.5;
    static final double     SLIDE_SPEED             = 0.5;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AeAnL1T/////AAABmTZwSvxDH0lVnljgy5pBgO8UdOWEaAiKyXqKqbABovZRTXBALSqlE0OHSRjJfjhCNHaOesi3e47zVd9aP/HvRyXToIZJvvzHxcjiDn4oYfAonwBGJdtJ2tbMZr91LDtH+xg3m1jiyA+UqCx0X9y0aKuKm4elTo5W9gayS0gW1T7bi1ww30yNScGGQABuYzqth/aUB50JDBnjpJV1Um1RljzpYq9RYQzOe1e+UTdCbznQSwEhFxsQmEM40oi6jjyUIWN+Ud3zcYvJ9hM2vbDOab/a1QZBEF6X/T/NCkeCsjQFj7yhA+hLFRMakOikLOb9ZDkrau7g+AxJ7M94puQiJzkkv6VTYn/C64jhur7CxzfP";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    double origAngle;
    Orientation turnAngles;
    BNO055IMU imu;
    double targetAngle;
    double difference;

    @Override
    public void runOpMode() {
      initVuforia();

      if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
          initTfod();
      } else {
          telemetry.addData("Sorry!", "This device is not compatible with TFOD");
      }

      if (tfod != null) {
          tfod.activate();
      }

      /*
       * Initialize the drive system variables.
       * The init() method of the hardware class does all the work here
       */

       imu = hardwareMap.get(BNO055IMU.class, "imu");
       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
       parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
       parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
       parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
       parameters.loggingEnabled = true;
       parameters.loggingTag = "IMU";
       parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
       imu.initialize(parameters);

       br  = hardwareMap.get(DcMotor.class, "br");
       fr  = hardwareMap.get(DcMotor.class, "fr");
       bl  = hardwareMap.get(DcMotor.class, "bl");
       fl  = hardwareMap.get(DcMotor.class, "fl" );
       foundationMotor  = hardwareMap.get(DcMotor.class, "foundation" );
       extend = hardwareMap.get(DcMotor.class, "extend");
       grab = hardwareMap.get(CRServo.class, "grab");

      // Send telemetry message to signify robot waiting;
      telemetry.addData("Status", "Resetting Encoders");    //
      telemetry.update();

      br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      foundationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      foundationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // Send telemetry message to indicate successful Encoder reset
      telemetry.addData("Status", "Encoders Reset. Ready");    //
      telemetry.update();

      // Wait for the game to start (driver presses PLAY)
      waitForStart();

      int stone_position = detect_skystone_position();
      int distanceFoundation = 80 + (stone_position - 1)*8;
      pickSkystone();
      turnLeft(90);
      dropSkystone();
      backward(DRIVE_SPEED, distanceFoundation);
      //forward(DRIVE_SPEED, distanceFoundation+24);
      turnLeft(-90);
      backward(DRIVE_SPEED, 8);
      Foundation(1, 0.5, 1.0);
      foundationMotor.setPower(0.25);
      forward(0.2, 40);
      Foundation(1, -0.5, 1.0);
      slideLeft(55);

      if (tfod != null) {
          tfod.shutdown();
      }
    }

    private void dropSkystone(){
      Foundation(1, -0.5, 1.0);
    }

    private void pickSkystone(){
      backward(DRIVE_SPEED, 10);
      Foundation(1, 0.5, 1.0);
      forward(DRIVE_SPEED, 10);
    }

    private int detect_skystone_position(){

      boolean skystoneFound = false;
      int stonePosition = 1;

      if(opModeIsActive()){
        backward(DRIVE_SPEED, 18);

        if (!skystoneFound) {
          if(isSkystone()){
            telemetry.addData("Position 1: ", "Skystone");
            telemetry.update();
            skystoneFound = true;
            slideRight(4);
          }
          else{
            telemetry.addData("Position 1: ", "Stone");
            slideLeft(10);
            telemetry.update();
          }

        }
        if (!skystoneFound) {
          if(isSkystone()){
            telemetry.addData("Position 2: ", "Skystone");
            telemetry.update();
            skystoneFound = true;
            stonePosition = 2;
            slideRight(4);
          }
          else{
            telemetry.addData("Position 2: ", "Stone");
            slideLeft(8);
            telemetry.update();
          }
        }
        if(!skystoneFound){
          stonePosition = 3;
        }
    }
    return stonePosition;
  }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
     public void forward(double speed, double inches){
       double dis = inches;
       encoderDrive(speed, -dis, dis, -dis, dis, 10.0);
     }
     public void backward(double speed, double inches){
       double dis = inches;
       encoderDrive(speed, dis, -dis, dis, -dis, 10.0);
     }

     public void slideRight(double inches){
       double dis = 5/4 * inches;
       encoderDrive(SLIDE_SPEED, -dis, -dis, dis, dis, 5.0);
     }
     public void slideLeft(double inches){
       double dis = 5/4 * inches;
       encoderDrive(SLIDE_SPEED, dis, dis, -dis, -dis, 5.0);
    }
    public void turnLeft(double degrees){
      double dis = (degrees * (3.14 * 23/ 360));
      encoderDrive(TURN_SPEED, dis, dis, dis, dis, 5.0);
    }
    //a --> a* 16.25PI/360
    public void turnRight(double degrees){
      double dis = (degrees * (3.14 * 23/ 360));
      encoderDrive(TURN_SPEED, -dis, -dis, -dis, -dis, 5.0);
    }

    public void AccurateTurn(double degrees){
        Orientation turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double origAngle = turnAngles.firstAngle;
        double targetAngle = origAngle + degrees;
        double difference = degrees;
        telemetry.addData("Difference", difference);
        telemetry.addData("Target angle", targetAngle);
        telemetry.addData("Current angle", turnAngles.firstAngle);
        telemetry.update();

        while (Math.abs(difference) > 0.95) {
          turnLeft(difference);
          turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          difference = targetAngle - turnAngles.firstAngle;
          telemetry.addData("Difference", difference);
          telemetry.addData("Target angle", targetAngle);
          telemetry.addData("Current angle", turnAngles.firstAngle);
          telemetry.update();
        }
      }

      private boolean isSkystone(){
        sleep(1000);
        if (tfod != null) {
          // getUpdatedRecognitions() will return null if no new information is available since
          // the last time that call was made.
          List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
          if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            telemetry.update();

            // step through the list of recognitions and display boundary info
            if(updatedRecognitions.size() != 1){
              telemetry.addData("# of Detected Stones ", String.valueOf(updatedRecognitions.size()));
              telemetry.update();
              return false;
            }

            for (Recognition recognition : updatedRecognitions) {
              telemetry.addData(String.format("label (%d)", 0), recognition.getLabel());
              telemetry.update();
              if(recognition.getLabel() == "Skystone"){
                return true;
              }
              else {
                return false;
              }
            }
          }
        }
        return false;
      }
      public void extend(double speed, double exmove, double timeoutS){
          int newexmovetarget;
          if (opModeIsActive()){
            newexmovetarget = extend.getCurrentPosition() + (int)(exmove * COUNTS_PER_INCH);
            extend.setTargetPosition(newexmovetarget);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            extend.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (extend.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Extend: ", "Running");
                telemetry.addData("exmove: ", String.valueOf(extend.isBusy()));
                telemetry.update();
            }
            // Display it for the driver.
            telemetry.addData("Extend: ", "Complete");
            telemetry.addData("exmove: ", String.valueOf(extend.isBusy()));
            telemetry.update();

            extend.setPower(0);
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          }
      }

    public void Foundation(double speed, double fmove, double timeoutS){
        int newfmovetarget;
        if (opModeIsActive()){
          newfmovetarget = foundationMotor.getCurrentPosition() + (int)(fmove * COUNTS_PER_INCH);
          foundationMotor.setTargetPosition(newfmovetarget);
          foundationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          runtime.reset();
          foundationMotor.setPower(Math.abs(speed));
          while (opModeIsActive() &&
                 (runtime.seconds() < timeoutS) &&
                 (foundationMotor.isBusy())) {

              // Display it for the driver.
              telemetry.addData("Foundation: ", "Running");
              telemetry.addData("fmove: ", String.valueOf(foundationMotor.isBusy()));
              telemetry.update();
          }
          // Display it for the driver.
          telemetry.addData("Foundation: ", "Complete");
          telemetry.addData("fmove: ", String.valueOf(foundationMotor.isBusy()));
          telemetry.update();

          foundationMotor.setPower(0);
          foundationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrive(double speed,
                             double flInches, double frInches, double blInches, double brInches,
                             double timeoutS) {
        int newbrtarget;
        int newbltarget;
        int newfrtarget;
        int newfltarget;
        boolean start_stop_opt = false;

        if (speed < 0.5 || brInches < 5) {
          start_stop_opt = false;
        } else {
          start_stop_opt = true;
        }
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int orgbrposition = br.getCurrentPosition();

            // Determine new target position, and pass to motor controller
            newbrtarget = br.getCurrentPosition() + (int)(brInches * COUNTS_PER_INCH);
            newbltarget = bl.getCurrentPosition() + (int)(blInches * COUNTS_PER_INCH);
            newfrtarget = fr.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
            newfltarget = fl.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
            br.setTargetPosition(newbrtarget);
            bl.setTargetPosition(newbltarget);
            fr.setTargetPosition(newfrtarget);
            fl.setTargetPosition(newfltarget);

            telemetry.addData("CurrentPos: ", String.valueOf(br.getCurrentPosition()));

            telemetry.addData("TargetPos: ", String.valueOf(newbrtarget));
            telemetry.update();



            // Turn On RUN_TO_POSITION
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            if (start_stop_opt) {
              br.setPower(Math.abs(speed*0.25));
              bl.setPower(Math.abs(speed*0.25));
              fr.setPower(Math.abs(speed*0.25));
              fl.setPower(Math.abs(speed*0.25));
            } else {
              // no point optimizing
              br.setPower(Math.abs(speed));
              bl.setPower(Math.abs(speed));
              fr.setPower(Math.abs(speed));
              fl.setPower(Math.abs(speed));
            }

            // telemetry.addData("fr: ", String.valueOf(fr.isBusy()));
            // telemetry.addData("bl: ", String.valueOf(bl.isBusy()));
            // telemetry.addData("fl: ", String.valueOf(fl.isBusy()));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (br.isBusy() && fr.isBusy() && fl.isBusy() && bl.isBusy())) {
                     int curbrposition = br.getCurrentPosition();
                     if (Math.abs(curbrposition - orgbrposition) < 0.05*(Math.abs(newbrtarget - orgbrposition))) {
                       br.setPower(Math.abs(speed*0.25));
                       bl.setPower(Math.abs(speed*0.25));
                       fr.setPower(Math.abs(speed*0.25));
                       fl.setPower(Math.abs(speed*0.25));
                     } else if (Math.abs(curbrposition - orgbrposition) < 0.1*(Math.abs(newbrtarget - orgbrposition))){
                       br.setPower(Math.abs(speed*0.5));
                       bl.setPower(Math.abs(speed*0.5));
                       fr.setPower(Math.abs(speed*0.5));
                       fl.setPower(Math.abs(speed*0.5));
                     } else if (Math.abs(curbrposition - orgbrposition) > 0.95*(Math.abs(newbrtarget - orgbrposition))) {
                       br.setPower(Math.abs(speed*0.25));
                       bl.setPower(Math.abs(speed*0.25));
                       fr.setPower(Math.abs(speed*0.25));
                       fl.setPower(Math.abs(speed*0.25));
                     } else if (Math.abs(curbrposition - orgbrposition) > 0.9*(Math.abs(newbrtarget - orgbrposition))) {
                       br.setPower(Math.abs(speed*0.25));
                       bl.setPower(Math.abs(speed*0.25));
                       fr.setPower(Math.abs(speed*0.25));
                       fl.setPower(Math.abs(speed*0.25));
                     } else {
                       br.setPower(Math.abs(speed));
                       bl.setPower(Math.abs(speed));
                       fr.setPower(Math.abs(speed));
                       fl.setPower(Math.abs(speed));
                     }

                // Display it for the driver.
                telemetry.addData("Path: ", "Running");
                telemetry.addData("br: ", String.valueOf(br.isBusy()));
                telemetry.update();
            }

            // Stop all motion;
            br.setPower(0);
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);

            // Turn off RUN_TO_POSITION


            telemetry.addData("FinalPos: ", String.valueOf((br.getCurrentPosition())/COUNTS_PER_INCH));

            //telemetry.addData("DistanceTraveled: ", String.valueOf(br.getCurrentPosition()-x));
            telemetry.update();

            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //     // optional pause after each move
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
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
