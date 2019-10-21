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
double PowerY;

    @Override public void runOpMode() {
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        extend = hardwareMap.get(DcMotor.class, "extend");
        while (opModeIsActive()) {
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

          // telemetry.addData("Status", "Run Time: " + runtime.toString());
          telemetry.addData("Motors","Arm extend power: (%.2f)" , PowerY);
          telemetry.addData("Written by", "deCoders Robotics Team");
          telemetry.update();
        }
    }
}
