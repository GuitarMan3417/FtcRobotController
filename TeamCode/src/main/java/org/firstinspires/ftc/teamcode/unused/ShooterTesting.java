package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name ="ShooterTest", group = "Inspection")
public class ShooterTesting extends LinearOpMode {
    private DcMotor M_LF;
    public void runOpMode(){
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_LF.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("System Initialized!");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                M_LF.setPower(1);
            }
            else if(gamepad1.b){
                M_LF.setPower(-1);
            }
            else{
                M_LF.setPower(0);
            }
            telemetry.addData("Motor Speed", M_LF.getPower());
            telemetry.update();
        }
    }
}
