/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.List;

/**
 * This is the Crater side OpMode
 * It does the following:
 * 1. Initialize
 * 2.
 *
 * Note: Each phone should have their own Vuforia license key.
 * Update the VUFORIA_KEY below with the key for the phone you are using
 * DeCoders Autonomou3s - zT
 * "AeAnL1T/////AAABmTZwSvxDH0lVnljgy5pBgO8UdOWEaAiKyXqKqbABovZRTXBALSqlE0OHSRjJfjhCNHaOesi3e47zVd9aP/HvRyXToIZJvvzHxcjiDn4oYfAonwBGJdtJ2tbMZr91LDtH+xg3m1jiyA+UqCx0X9y0aKuKm4elTo5W9gayS0gW1T7bi1ww30yNScGGQABuYzqth/aUB50JDBnjpJV1Um1RljzpYq9RYQzOe1e+UTdCbznQSwEhFxsQmEM40oi6jjyUIWN+Ud3zcYvJ9hM2vbDOab/a1QZBEF6X/T/NCkeCsjQFj7yhA+hLFRMakOikLOb9ZDkrau7g+AxJ7M94puQiJzkkv6VTYn/C64jhur7CxzfP"
 * Intake Mechanism - OQ
 * "AU4JLOL/////AAABmXgaGQRo4kuAne7wIdQUsSE9SqyXCy4qc7LaVZuWQ2VNX2Ik6xkyPId7LaO8XhSORtSlv+KcD+XJxtHVsnaoY/lOit2dQ+IKMA1aw0Xgunr7xB+Qrxl7YTnJhpE+Wih4f+BbG0GkuctRKVCV1Kaijc0Z9qay8LroxRJNVuCX+rdJ5t6c2QlPjPvXubQPnWKvF/KB1r0c1AShQy/zD5OlwWzMOl+cXrnZ3BXZW31ciyfjDEMinF4U1gAy2S662nZxRogUI+eHs4fHsizUKliqFHAJTlPfRfc+cJSD/mWltMatV4vPezeD2F+gPxrXb9dLRutDoL/qdIXAXflZjxkl94aiRpzWu3hUZzKnq11NnU7Y"
 * PK
 * "ARwyrbr/////AAABmfOPvaqdyUItuak5WGND/d95mKYbin0FHuTmGx8D7ZVA+I/RXfFH9Cnx09qKG9SgWa0pQrQxf/72VPuH4D0T3ZUqADFUQbI518dIPmND3a7T3hgiet1TD5NrBODd0Hb5yd+SQup5z7ogbsBtkI3CSviXAoTVqMiZOjxMYC340zT1rnhXLW1L3yeRhTinmhzezNIr/fSut7bJyuVzShQTZquDv8eAhcoQPJ1LIDLR+kkwoo2aCyvcUiEBSWG5vfuquhFtShp0Bc3dUhpeZlTiJiMuKThM56jmFrGrRR8BdkefwDtsmxPhAGmVomWH6AVIqN3s8y57S+Fv/291q/AFrhw3ML0eGpBgcj6n2htcRyxu"
 */


@Autonomous(name = "Depot Autonomous", group = "Linear OpMode")
public class OLDCODEDEPOTAUTONOMOUS extends LinearOpMode {

    private static final int GOLD_LEFT = 1;
    private static final int GOLD_CENTER = 2;
    private static final int GOLD_RIGHT = 3;
    int goldMineralLeft;
    private DcMotor LeftDriveFront;
    private DcMotor RightDriveFront;
    private DcMotor LeftDriveBack;
    private DcMotor RightDriveBack;
    private DcMotor lift;
    public CRServo marker = null;
    public float current_angle;
    private DistanceSensor heightSensor;

    static final double COUNTS_PER_MOTOR_REV  = 1440 ; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION  = 1.0;   // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;  // For figuring circumference
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_2       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED           = 1.0;
    static final double TURN_SPEED            = 1.0;
    static final double SLIDE_SPEED           = 1.0;

    Orientation angles;
    double origAngle;
    BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    public void orientationChecker(double angleTarget){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double newAngle;
        double difference;
        int checknum = 0;
        newAngle = angles.firstAngle;
        difference = newAngle-angleTarget;

        while(checknum<=3 && difference!=0){

            TurnRight(difference);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            newAngle = angles.firstAngle;
            difference = newAngle-angleTarget;
            telemetry.addData("NewAngle", newAngle);
            telemetry.addData("Difference", difference);
            telemetry.addData("Original Angle", origAngle);
            telemetry.addData("AngleTarget", angleTarget);
            telemetry.update();
            telemetry.clear();
            checknum++;
        }
    }

    public void stopAllMotors(){
        LeftDriveBack.setPower(0);
        RightDriveBack.setPower(0);
        LeftDriveFront.setPower(0);
        RightDriveFront.setPower(0);
        lift.setPower(0);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        marker = hardwareMap.get(CRServo.class, "marker");

        LeftDriveFront  = hardwareMap.get(DcMotor.class, "LeftDriveFront");
        RightDriveFront = hardwareMap.get(DcMotor.class, "RightDriveFront");
        LeftDriveBack    = hardwareMap.get(DcMotor.class, "LeftDriveBack");
        RightDriveBack  = hardwareMap.get(DcMotor.class, "RightDriveBack");

        LeftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        RightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        RightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        LeftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LeftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lift  = hardwareMap.get(DcMotor.class, "lift");
        heightSensor = hardwareMap.get(DistanceSensor.class, "Height");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        stopAllMotors();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status","Waiting for User ");
        telemetry.update();
        waitForStart();
        runtime.reset();

        /**
         * Landing code
         */
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angle_at_top = angles.firstAngle;

          while (opModeIsActive()) {
              encoderLift();
              break;
          }
        telemetry.addData("Angle at top", angle_at_top);
        telemetry.update();


        /**
         * Sampling
         */

        if (opModeIsActive()) {
            switch (getGoldPosition2()) {
                case GOLD_LEFT:
                    telemetry.addData("Gold Pos", "Left");
                    telemetry.update();
                    slideRight(10);
                    Forwards(26);
                    slideRight(26);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    current_angle = angles.firstAngle;
                    AccurateTurn(45 - current_angle+angle_at_top);
                    slideRight(30);
                    timedSpin(500);
                    Backwards(80);
                    break;
                case GOLD_CENTER:
                    telemetry.addData("Gold Pos", "Center");
                    telemetry.update();
                    slideRight(10);
                    Forwards(7);
                    slideRight(37);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    current_angle = angles.firstAngle;
                    AccurateTurn(45 - current_angle + angle_at_top);
                    slideRight(15);
                    timedSpin(500);
                    Backwards(70);
                    break;


                case GOLD_RIGHT:
                    telemetry.addData("Gold Pos", "Right");
                    telemetry.update();
                    slideRight(12);
                    Backwards(8);
                    slideRight(35);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    current_angle = angles.firstAngle;
                    AccurateTurn(44 - current_angle + angle_at_top);
                    Forwards(15);
                    timedSpin(500);
                    slideLeft(4);
                    Backwards(76);
                    break;
            }
        }


    }
    public void timedSpin(long timeMS){
        if(timeMS >= 0){
            marker.setPower(-1);
        }
        else{
            marker.setPower(1);
        }
        sleep(Math.abs(timeMS));
        marker.setPower(0);
    }
    public void encoderDrive (double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = LeftDriveFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = RightDriveFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = LeftDriveBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = RightDriveBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            LeftDriveFront.setTargetPosition(newLeftFrontTarget);
            RightDriveFront.setTargetPosition(newRightFrontTarget);
            LeftDriveBack.setTargetPosition(newLeftBackTarget);
            RightDriveBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            LeftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftDriveBack.setPower(Math.abs(speed));
            LeftDriveFront.setPower(Math.abs(speed));
            RightDriveBack.setPower(Math.abs(speed));
            RightDriveFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && ( LeftDriveBack.isBusy() &&  RightDriveBack.isBusy())) {


            }

            // Stop all motion;
            LeftDriveBack.setPower(0);
            RightDriveBack.setPower(0);
            LeftDriveFront.setPower(0);
            RightDriveFront.setPower(0);
            lift.setPower(0);
            //Set to RUN_USING_ENCODER
            LeftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void encoderLift(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angle_at_top = angles.firstAngle;
        lift.setPower(1.0);
        sleep(7800);
        telemetry.addData("Landed", "ground?");
        telemetry.update();
        lift.setPower(0);
        Backwards(8.5);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        AccurateTurn(angles.firstAngle - angle_at_top);
        slideRight(11);
    }
    public void Backwards(double distance){
        encoderDrive(DRIVE_SPEED,distance,distance,distance,distance, 5);
    }
    public void Forwards(double distance){
        encoderDrive(DRIVE_SPEED, -distance, -distance, -distance, -distance, 5);
    }
    public void TurnLeft(double a){
        double degrees = a * 24/90;
        encoderDrive(TURN_SPEED, degrees, -degrees, degrees,- degrees,5);
    }
    public void TurnRight(double a){
        double degrees = a * 24/90;
        encoderDrive(TURN_SPEED, -degrees, degrees, -degrees, degrees, 5);
    }
    public void slideRight(double distance){
        encoderDrive(SLIDE_SPEED,-distance,distance,distance,-distance,5);
    }
    public void slideLeft(double distance){
        encoderDrive(SLIDE_SPEED,distance,-distance,-distance,distance,5);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void AccurateTurn(double degrees){
        degrees *= -1;
        Orientation turnAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double origAngle = turnAngles.firstAngle;
        double targetAngle = origAngle + degrees;
        double difference = degrees;
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

    /**
     * Runs the object detection algorithm and returns the position of the gold mineral
     * @return
     */
    // Update the developer key from top level comment
    private static final String VUFORIA_KEY = "AeAnL1T/////AAABmTZwSvxDH0lVnljgy5pBgO8UdOWEaAiKyXqKqbABovZRTXBALSqlE0OHSRjJfjhCNHaOesi3e47zVd9aP/HvRyXToIZJvvzHxcjiDn4oYfAonwBGJdtJ2tbMZr91LDtH+xg3m1jiyA+UqCx0X9y0aKuKm4elTo5W9gayS0gW1T7bi1ww30yNScGGQABuYzqth/aUB50JDBnjpJV1Um1RljzpYq9RYQzOe1e+UTdCbznQSwEhFxsQmEM40oi6jjyUIWN+Ud3zcYvJ9hM2vbDOab/a1QZBEF6X/T/NCkeCsjQFj7yhA+hLFRMakOikLOb9ZDkrau7g+AxJ7M94puQiJzkkv6VTYn/C64jhur7CxzfP";

    // {@link #vuforia} is the variable we will use to store our instance of the Vuforia
    // localization engine.
    private VuforiaLocalizer vuforia;

    // {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
    // Detection engine.
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public int getGoldPosition2() {

        /**
         * TensorFlow initial variables
         */
        int goldMineralX = -1;
        int silverMineral1X = -1;
        int silverMineral2X = -1;
        goldMineralLeft = -1;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        //waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null && getRuntime()>2) {
                tfod.activate();
            }
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 4)) {
            //telemetry.addData("Elapsed Time", runtime.toString());
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                     if (updatedRecognitions.size() == 2) {
                         for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralLeft = (int) recognition.getLeft();
                                telemetry.addData("Gold",goldMineralLeft);
                                telemetry.update();
                            } else {
                               return GOLD_LEFT;
                            }
                        }
                        if (goldMineralLeft > 739.5) {
                            return GOLD_RIGHT;
                        } else {
                            return GOLD_CENTER;
                        }



                    //     // If 1 gold and 1 silver is detected

                    //     if(goldMineralX != -1 && silverMineral1X != -1){
                    //         telemetry.addData("Detected Objects", "Gold and Silver");
                    //         if (goldMineralX > silverMineral1X) {
                    //             telemetry.addData("Gold Mineral Position", "Right");
                    //             return GOLD_RIGHT;
                    //         } else{
                    //             telemetry.addData("Gold Mineral Position", "Center");
                    //             return GOLD_CENTER;

                    //         }



                    //     }

                    }

                    if((updatedRecognitions.size() == 1) || (updatedRecognitions.size() >=3)) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralLeft = (int) recognition.getLeft();
                            } else {
                               return GOLD_LEFT;
                            }
                        }
                        if (goldMineralLeft > 739.5) {
                            return GOLD_RIGHT;
                        } else {
                            return GOLD_CENTER;
                        }
                    }
                    else if(updatedRecognitions.size()==0 && runtime.seconds()>=4){
                        return GOLD_LEFT;
                    }

                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return GOLD_LEFT;
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

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.2 ;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
