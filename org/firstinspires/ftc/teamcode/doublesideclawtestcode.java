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

@TeleOp(name="doublesideclawtestcode", group ="Concept")
// @Disabled

public class doublesideclawtestcode extends LinearOpMode {
private DcMotor extend;
private CRServo grab;
double PowerX;
double PowerY;

    @Override public void runOpMode() {
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        extend = hardwareMap.get(DcMotor.class, "extend");
        grab = hardwareMap.get(CRServo.class, "grab");
        while (opModeIsActive()) {
          //Arm code (extend)
          if(gamepad1.a){
            PowerX = 1;
          }
          else if(gamepad1.y){
            PowerX = -1;
          }
          else{
            PowerX = 0;
          }

          PowerY = gamepad1.right_trigger - gamepad1.left_trigger;
          extend.setPower(PowerY);
          grab.setPower(PowerX);
          telemetry.addData("Motors","Arm extend power: (%.2f)",PowerY);
          telemetry.addData("Written by", "deCoders Robotics Team");
          telemetry.update();
        }
    }
}
