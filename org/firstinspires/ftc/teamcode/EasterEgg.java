package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Easter Egg", group="Linear Opmode")

public class EasterEgg extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private double sensitivity = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

			      double LFP;
            double RFP;
            double RBP;
            double LBP;

            double D = gamepad1.left_stick_y;
            double S = gamepad1.left_stick_x;
            double T = gamepad1.right_stick_x;

            LFP = T - S - D;
            RFP = T + S - D;
            LBP = T - S + D;
            RBP = T + S + D;


			// This can be anywhere from 0.5 to 1.
			// Anything lower than 0.5 is very slow, which defeats the purpose of the speedy drivetrain.
			// Anything above 1 will break the code, please don't do this.
      if(gamepad1.a){
        if(sensitivity<1){
          sensitivity = sensitivity + 0.1;
          sleep(200);
        }
      }
      else if(gamepad1.b){
        if(sensitivity>0.6){
          sensitivity = sensitivity - 0.1;
          sleep(200);
        }
      }
			LFP *= sensitivity;
			LBP *= sensitivity;
			RFP *= sensitivity;
			RBP *= sensitivity;
			// x *= y is a fancy way of redifining variable x so that x = x*y
			// Same with +=, -=, and /=
			// Just something to remember...

            if(T != 0 && (D != 0 || S != 0)){
                fl.setPower(LFP/2);
                bl.setPower(LBP/2);
                fr.setPower(RFP/2);
                br.setPower(RBP/2);

            }
            else{
                fl.setPower(LFP);
                bl.setPower(LBP);
                fr.setPower(RFP);
                br.setPower(RBP);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors","Left Front Power: (%.2f), Right Front Power: (%.2f), Left Back Power: (%.2f), Right Back Power: (%.2f), Sensitivity: (%.2f)", fl.getPower(), fr.getPower(), bl.getPower(), br.getPower(), sensitivity);
            telemetry.addData("Written by", "deCoders Robotics Team");
            telemetry.update();
        }
    }
}
