package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="AutonomousFoundation", group="Pushbot")
public class AutonomousFoundation extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    private DcMotor br;
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor foundationMotor;
                private DcMotor extend;

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.5;

    double origAngle;
    Orientation turnAngles;
    BNO055IMU imu;
    double targetAngle;
    double difference;

    @Override
    public void runOpMode() {

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()){
        slideLeft(20);
        slideRight(20);
        /*
        Orientation runangles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float beginangle = runangles.firstAngle;
        forward(27);
        slideLeft(30);
        turnRight(90);
        Foundation(1, 0.75, 2.0);
        foundationMotor.setPower(1);
        slideRight(44);


        turnLeft(270);
        forward(6.9);
        foundationMotor.setPower(-1);
        Orientation intermediateangles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float superangle = intermediateangles.firstAngle;
        turnRight(superangle - beginangle);
        slideRight(50);



        */
        telemetry.addData("Path", "Complete");
        telemetry.update();
      }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
     public void forward(double inches){
       double dis = inches;
       encoderDrive(DRIVE_SPEED, -dis, dis, -dis, dis, 5.0);
     }
     public void backward(double inches){
       double dis = inches;
       encoderDrive(DRIVE_SPEED, dis, -dis, dis, -dis, 5.0);
     }
     public void slideRight(double inches){
       double dis = 5/4 * inches;
       encoderDrive(DRIVE_SPEED, -dis, -dis, dis, dis, 5.0);
     }
     public void slideLeft(double inches){
       double dis = 5/4 * inches;
       encoderDrive(DRIVE_SPEED, dis, dis, -dis, -dis, 5.0);
    }
    public void turnLeft(double degrees){
      double dis = (degrees * 51.05/360);
      encoderDrive(TURN_SPEED, dis, dis, dis, dis, 5.0);
    }
    public void TurnLeft2(double degrees){
      double dis = (degrees * 51.05/360);
      encoderDrive(1.0, dis, dis, dis, dis, 5.0);
    }
    //a --> a* 16.25PI/360
    public void turnRight(double degrees){
      double dis = (degrees * 51.05/360);
      encoderDrive(TURN_SPEED, -dis, -dis, -dis, -dis, 5.0);
    }

    public void AccurateTurn(double degrees){
        Orientation turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double origAngle = turnAngles.firstAngle;
        double targetAngle = origAngle + degrees;
        double difference = degrees;
        while (Math.abs(difference) > 1) {
            turnRight(difference * 0.9);
            turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            difference = targetAngle - turnAngles.firstAngle;
            telemetry.addData("Difference", difference);
            telemetry.addData("Target angle", targetAngle);
            telemetry.addData("Current angle", turnAngles.firstAngle);
            telemetry.update();
            sleep(5000);
        }
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
                             double flInches, double frInches,double blInches, double brInches,
                             double timeoutS) {
        int newbrtarget;
        int newfrtarget;
        int newfltarget;
        int newbltarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newbrtarget = br.getCurrentPosition() + (int)(brInches * COUNTS_PER_INCH);
            newfrtarget = fr.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
            newfltarget = fl.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
            newbltarget = bl.getCurrentPosition() + (int)(blInches * COUNTS_PER_INCH);
            br.setTargetPosition(newbrtarget);
            fr.setTargetPosition(newfrtarget);
            fl.setTargetPosition(newfltarget);
            bl.setTargetPosition(newbltarget);

            telemetry.addData("CurrentPos: ", String.valueOf(br.getCurrentPosition()));

            telemetry.addData("TargetPos: ", String.valueOf(newbrtarget));
            telemetry.update();



            // Turn On RUN_TO_POSITION
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            br.setPower(Math.abs(speed));
            fr.setPower(Math.abs(speed));
            bl.setPower(Math.abs(speed));
            fl.setPower(Math.abs(speed));

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
            //sleep(2000);

            //telemetry.addData("DistanceTraveled: ", String.valueOf(br.getCurrentPosition()-x));
            telemetry.update();
            //sleep(2000);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //     // optional pause after each move
        }
    }

}
