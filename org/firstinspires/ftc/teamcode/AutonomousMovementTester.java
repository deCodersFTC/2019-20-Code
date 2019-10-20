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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.lang.Math;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.List;

@Autonomous(name = "AutonomousMovementTester" ,group = "Testers")

public class AutonomousMovementTester extends LinearOpMode {

    public DcMotor  fl;
    public DcMotor  fr;
    public DcMotor  bl;
    public DcMotor  br;
    public DistanceSensor heightSensor;

    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;
    static final double     SLIDE_SPEED             = 0.6;

    Orientation angles;
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
            bl.setPower(Math.abs(speed));
            fl.setPower(Math.abs(speed));
            br.setPower(Math.abs(speed));
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
        distance = (distance *= 0.95) / 2;
        encoderDrive(DRIVE_SPEED,-distance,-distance,distance,distance, 1);
    }
    public void Forwards(double distance){
      distance = (distance *= 0.95) / 2;
        encoderDrive(DRIVE_SPEED, distance, distance, -distance, -distance, 1);
    }
    public void TurnLeft(double a){
        double degrees = a * 24/90;
        encoderDrive(TURN_SPEED, -degrees, -degrees, -degrees,-degrees,1);
    }
    public void TurnRight(double a){
        double degrees = a * 24/90;
        encoderDrive(TURN_SPEED, degrees, degrees, degrees, degrees, 1);
    }
    public void slideRight(double distance){
      distance = (distance *= 0.95) / 2;
        encoderDrive(SLIDE_SPEED,-distance,distance,-distance,distance,1);
    }
    public void slideLeft(double distance){
      distance = (distance *= 0.95) / 2;
        encoderDrive(SLIDE_SPEED,distance,-distance,distance,-distance,1);
    }
    public void stopAllMotors(){
      encoderDrive(0,0,0,0,0,0.5);
    }
    public void SAM(){
      stopAllMotors();
    }

    public void AccurateTurn(double degrees){
        degrees *= -1;
        turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        origAngle = turnAngles.firstAngle;
        targetAngle = origAngle + degrees;
        difference = degrees;
        while (Math.abs(difference) > 1) {
            TurnLeft(difference * 0.9);
            turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            difference = targetAngle - turnAngles.firstAngle;
            // telemetry.addData("Difference", difference);
            // telemetry.addData("New angle", targetAngle);
            // telemetry.addData("Angles.firstAngle", turnAngles.firstAngle);
            // telemetry.update();

        }
  }

    public void runOpMode(){
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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            Forwards(1);
            SAM();
            Backwards(1);
            SAM();
            slideRight(1);
            SAM();
            slideLeft(1);
            SAM();
            AccurateTurn(360);
            SAM();
            AccurateTurn(360);
            SAM();
            break;

        }
    }
    // todo: write your code here
}
