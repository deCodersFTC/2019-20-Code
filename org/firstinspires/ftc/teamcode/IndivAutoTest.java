package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Indiv Auto Test", group="Pushbot")
public class IndivAutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    private DcMotor fl;
    // private DcMotor fr;
    // private DcMotor br;
    // private DcMotor bl;

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
         fl  = hardwareMap.get(DcMotor.class, "fl");
         // fr  = hardwareMap.get(DcMotor.class, "fr");
         // bl  = hardwareMap.get(DcMotor.class, "bl");
         // br  = hardwareMap.get(DcMotor.class, "br" );

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  100, -48, 48, -48, 10);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, 12, 12, 12, 4.0); // S3: Turn 12 Inches with 4 Sec timeout



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double flInches, double frInches,double blInches, double brInches,
                             double timeoutS) {
        int newfltarget;
        // int newfrtarget;
        // int newbrtarget;
        // int newbltarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfltarget = fl.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
            // newfrtarget = fr.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
            // newbrtarget = br.getCurrentPosition() + (int)(brInches * COUNTS_PER_INCH);
            // newbltarget = bl.getCurrentPosition() + (int)(blInches * COUNTS_PER_INCH);
            fl.setTargetPosition(newfltarget);

            telemetry.addData("CurrentPos: ", String.valueOf(fl.getCurrentPosition()));

            telemetry.addData("TargetPos: ", String.valueOf(newfltarget));
            telemetry.update();
            //sleep(5000);
            // fr.setTargetPosition(newfrtarget);
            // br.setTargetPosition(newbrtarget);
            // bl.setTargetPosition(newbltarget);

            // Turn On RUN_TO_POSITION
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fl.setPower(Math.abs(speed));
            // fr.setPower(Math.abs(speed));
            // bl.setPower(Math.abs(speed));
            // br.setPower(Math.abs(speed));

            // telemetry.addData("fr: ", String.valueOf(fr.isBusy()));
            // telemetry.addData("bl: ", String.valueOf(bl.isBusy()));
            // telemetry.addData("br: ", String.valueOf(br.isBusy()));
            //sleep(2000);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (fl.isBusy()/* && fr.isBusy() && br.isBusy() && bl.isBusy())*/)) {

                // Display it for the driver.
                telemetry.addData("Path: ", "Running");
                telemetry.addData("fl: ", String.valueOf(fl.isBusy()));
                telemetry.update();
            }

            // Stop all motion;
            fl.setPower(0);
            // fr.setPower(0);
            // br.setPower(0);
            // bl.setPower(0);

            // Turn off RUN_TO_POSITION
            sleep(5000);
            telemetry.addData("FinalPos: ", String.valueOf((fl.getCurrentPosition())/COUNTS_PER_INCH));


            //telemetry.addData("DistanceTraveled: ", String.valueOf(fl.getCurrentPosition()-x));
            telemetry.update();
            sleep(5000);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
