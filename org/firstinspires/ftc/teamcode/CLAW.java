package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="CLAW", group ="Concept")
// @Disabled

public class CLAW extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor fl;
  private DcMotor fr;
  private DcMotor bl;
  private DcMotor br;
  private double sensitivity = 1;
    private CRServo grab;
    // private DcMotor lift;
    private DcMotor extend;
    double PowerX;
    double PowerY;
    @Override public void runOpMode() {
        grab  = hardwareMap.get(CRServo.class, "grab");
        extend = hardwareMap.get(DcMotor.class, "extend");
        telemetry.addData(">", "Press Play to start tracking");
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
        while (opModeIsActive()) {
          //Claw code
          if(gamepad2.x){
            PowerX = 0.5;
          }
          else if(gamepad2.b){
            PowerX = -0.5;
          }
          else{
            PowerX = 0;
          }

          //Arm code (extend)
          /*
          if(gamepad2.right_trigger != 0){
            PowerY = gamepad2.right_trigger/2;
          }
          if(gamepad2.left_trigger != 0){
            PowerY = gamepad2.left_trigger/2;
          }
          */
          if(gamepad2.y){
            PowerY = 0.5;
          }
          else if(gamepad2.a){
            PowerY = -0.5;
          }
          else{
            PowerY = 0;
          }
          extend.setPower(PowerY);
          grab.setPower(PowerX);



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

          // telemetry.addData("Status", "Run Time: " + runtime.toString());
          telemetry.addData("Motors","Claw/Grabber Power: (%.2f), Arm extend power: (%.2f)" ,PowerX, PowerY);
          telemetry.addData("Written by", "deCoders Robotics Team");
          telemetry.update();
        }
    }
}
