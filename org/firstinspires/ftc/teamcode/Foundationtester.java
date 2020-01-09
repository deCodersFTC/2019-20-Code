package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Foundation Mover", group="Linear Opmode")

public class FoundationTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor extend;
    private CRServo claw;
    private Servo lift;
    private Servo LeftFoundation;
    private Servo RightFoundation;
    private double sensitivity = 0.8;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        claw = hardwareMap.get(CRServo.class, "claw");
        lift = hardwareMap.get(Servo.class, "lift");
        LeftFoundation = hardwareMap.get(Servo.class, "Left Foundation");
        RightFoundation = hardwareMap.get(Servo.class, "Right Foundation");
        extend = hardwareMap.get(DcMotor.class, "extend");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        extend.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

          if(gamepad1.dpad_up){
            fl  = hardwareMap.get(DcMotor.class, "fl");
            fr = hardwareMap.get(DcMotor.class, "fr");
            bl  = hardwareMap.get(DcMotor.class, "bl");
            br = hardwareMap.get(DcMotor.class, "br");

            fl.setDirection(DcMotor.Direction.FORWARD);
            fr.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.FORWARD);
            br.setDirection(DcMotor.Direction.REVERSE);
          }
          else if(gamepad1.dpad_down){
            fl  = hardwareMap.get(DcMotor.class, "br");
            fr = hardwareMap.get(DcMotor.class, "bl");
            bl  = hardwareMap.get(DcMotor.class, "fr");
            br = hardwareMap.get(DcMotor.class, "fl");

            fl.setDirection(DcMotor.Direction.FORWARD);
            fr.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.FORWARD);
            br.setDirection(DcMotor.Direction.REVERSE);
          }

            double LFP;
            double RFP;
            double RBP;
            double LBP;

            double D = gamepad1.left_stick_y;
            double S = -gamepad1.left_stick_x;
            double T = -gamepad1.right_stick_x;

            // LFP = T - S - D;
            // RFP = T + S - D;
            // LBP = T - S + D;
            // RBP = T + S + D;

            LFP = D + S + T;
            RFP = D - S - T;
            LBP = D - S + T;
            RBP = D + S - T;


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
        if(sensitivity>0.4){
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

            if(gamepad1.left_bumper){
              LeftFoundation.setPosition(0);
              RightFoundation.setPosition(0);
            }
            else if(gamepad1.right_bumper){
              LeftFoundation.setPosition(1);
              RightFoundation.setPosition(1);
            }


             if(gamepad2.left_bumper){
              lift.setPosition(1);


            }
            else{
              lift.setPosition(0);
            }

            if (gampad2.a){
              extend.setPower(0.5);
            } elif (gamepad2.b){
              extend.setPower(-0.5)
            } else{
              extend.setPower(0);
            }

            claw.setPower((gamepad2.right_trigger-gamepad2.left_trigger));


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors","Left Front Power: (%.2f), Right Front Power: (%.2f), Left Back Power: (%.2f), Right Back Power: (%.2f), Sensitivity: (%.2f)", fl.getPower(), fr.getPower(), bl.getPower(), br.getPower(), sensitivity);
            telemetry.addData("Extend Power", extend.getPower());
            telemetry.addData("Written by", "deCoders Robotics Team");
            telemetry.update();
        }
    }
}
