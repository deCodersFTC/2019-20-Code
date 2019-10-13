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
private CRServo grab;
// private DcMotor lift;
private DcMotor extend;
double PowerX;
double PowerY;

    @Override public void runOpMode() {
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        grab  = hardwareMap.get(CRServo.class, "grab");
        extend = hardwareMap.get(DcMotor.class, "extend");
        while (opModeIsActive()) {
          //Claw code
          if(gamepad1.x){
            PowerX = 1;
          }
          else if(gamepad1.b){
            PowerX = -1;
          }
          else{
            PowerX = 0;
          }

          //Arm code (extend)
          if(gamepad1.y){
            PowerY = 0.5;
          }
          else if(gamepad1.a){
            PowerY = -1;
          }
          else{
            PowerY = 0;
          }
          extend.setPower(PowerY);
          grab.setPower(PowerX);

          // telemetry.addData("Status", "Run Time: " + runtime.toString());
          telemetry.addData("Motors","Claw/Grabber Power: (%.2f), Arm extend power: (%.2f)" ,PowerX, PowerY);
          telemetry.addData("Written by", "deCoders Robotics Team");
          telemetry.update();
        }
    }
}
