package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name="SimpleMotorTest", group = "Inspection")
public class SimpleMotorTest extends LinearOpMode {
    private DcMotor M_LR;
    private DcMotor M_RR;
    private DcMotor M_LF;
    private DcMotor M_RF;
    @Override
    public void runOpMode(){
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        telemetry.addLine("System Initialized!");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            for(double i = 0; i < 1; i = i += 0.001){
                Forward(i);
                telemetry.addData("Forward", i);
                telemetry.update();
            }
            sleep(1000);
            for(double i = 0; i < 1; i += 0.001){
                Backward(i);
                telemetry.addData("Backward", i);
                telemetry.update();
            }
            sleep(1000);

        }
    }
    public void Forward(double speed){
        M_LR.setPower(speed *-1);
        M_RR.setPower(speed);
        M_LF.setPower(speed * -1);
        M_RF.setPower(speed);
    }
    public void Backward(double speed){
        M_LR.setPower(speed);
        M_RR.setPower(speed *-1);
        M_LF.setPower(speed);
        M_RF.setPower(speed *-1);
    }
}
