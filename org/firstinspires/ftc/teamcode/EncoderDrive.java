/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="EncoderDrive", group="Linear Opmode")

public class EncoderDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    //private DcMotor lift;
    //private DcMotor extend;
    //float PowerX;
    //float PowerY;

    static final double COUNTS_PER_MOTOR_REV  = 560 ; // REV HD HEX Motor Encoder: Counts per rotation of the output shaft. More info can be found here: http://www.revrobotics.com/content/docs/Encoder-Guide.pdf
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
            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
        }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");



        // lift  = hardwareMap.get(DcMotor.class, "ba");
        // extend = hardwareMap.get(DcMotor.class, "ta");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
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

        // lift.setDirection(DcMotor.Direction.FORWARD);
        // extend.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
          Forwards(5);
          // Backwards(5);
          // slideRight(5);
          // slideLeft(5);
          // TurnLeft(90);
          // TurnRight(90);
            //
            // // Setup a variable for each drive wheel to save power level for telemetry
            // double LFP;
            // double RFP;
            // double RBP;
            // double LBP;
            //
            // double D = gamepad1.left_stick_y;
            // double S = gamepad1.left_stick_x;
            // double T = gamepad1.right_stick_x;
            // //Forward: LF: 0.5 RF: -0.5 RB: - 0.5 LB: 0.5
            // // Choose to drive using either Tank Mode, or POV Mode
            // // Comment out the method that's not used.  The default below is POV.
            //
            // // POV Mode uses left stick to go forward, and right stick to turn.
            // // - This uses basic math to combine motions and is easier to drive straight.
            // LFP = T + S + D;
            // RFP = T + S - D;
            // LBP = T - S + D;
            // RBP = T - S - D;
            //
            // // Tank Mode uses one stick to control each wheel.
            // // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // // leftPower  = -gamepad1.left_stick_y ;
            // // rightPower = -gamepad1.right_stick_y ;
            //
            // // Send calculated power to wheel
            // if(turn != 0 && (drive != 0 || slide != 0)){
            //     fl.setPower(LFP/2);
            //     bl.setPower(LBP/2);
            //     fr.setPower(RFP/2);
            //     rl.setPower(RBP/2);
            //
            // }
            // else{
            //     fl.setPower(LFP);
            //     bl.setPower(LBP);
            //     fr.setPower(RFP);
            //     br.setPower(RBP);
            // }
            // /*
            // PowerX = gamepad2.left_stick_y;
            // PowerY = gamepad2.right_stick_y;
            // extend.setPower(PowerX);
            // lift.setPower(PowerY);
            // */
            // // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors","Left Front Power: (%.2f), Right Front Power: (%.2f), Left Back Power: (%.2f), Right Back Power: (%.2f)", fl.getPower(), fr.getPower(), bl.getPower(), br.getPower());
            telemetry.addData("Written by", "deCoders Robotics Team");
            telemetry.update();
        }


    }
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
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && ( bl.isBusy() &&  br.isBusy())) {


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
      //                         lf         rf         lb         rb
        encoderDrive(DRIVE_SPEED, -distance, -distance, distance, distance, 5);
    }
    public void Forwards(double distance){
        encoderDrive(DRIVE_SPEED, distance, distance, -distance, -distance, 5);
    }
    public void TurnLeft(double a){
        double degrees = a * 24/90;
        encoderDrive(TURN_SPEED, -degrees, -degrees, -degrees,-degrees,5);
    }
    public void TurnRight(double a){
        double degrees = a * 24/90;
        encoderDrive(TURN_SPEED, degrees, degrees, degrees, degrees, 5);
    }
    public void slideRight(double distance){
        encoderDrive(SLIDE_SPEED,-distance, distance, -distance, distance, 5);
    }
    public void slideLeft(double distance){
        encoderDrive(SLIDE_SPEED, distance, -distance, distance, -distance,5);
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
}
