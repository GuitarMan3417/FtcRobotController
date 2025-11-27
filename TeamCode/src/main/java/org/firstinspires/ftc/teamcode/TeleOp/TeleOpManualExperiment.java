package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;
@TeleOp(name="ExperimentalOp", group = "TeleOp")
public class TeleOpManualExperiment extends LinearOpMode {
    int lastTime = 0;
    int lastPos = 0;
    DcMotor M_AIN, M_S0, M_S1, M_bl, M_LF, M_RF, M_LR, M_RR;
    Servo SVR_L0, SVR_L1;
    //System Timer
    Timer motorTimer = new Timer();
    TimerTask motorAct = new TimerTask() {
        @Override
        public void run() {
            M_bl.setPower(1);
        }
    };
    private int shootingAngle = 0; //Initial shooting angle
    private int angleAdd = 1; //Servo angle addition value

    @Override
    public void runOpMode(){
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN"); //Artifact In
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0"); //Spin up Motor
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1"); //Shoot Motor
        M_bl = hardwareMap.get(DcMotor.class, "M_bl"); //Blocking motor (Core Hex Recommended)
        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        M_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad2.right_bumper){
                shootingAngle += angleAdd;
            }
            if(gamepad2.left_bumper){
                shootingAngle -= angleAdd;
            }
            shootingAngle = Math.max(0, Math.min(180, shootingAngle));
            SVR_L0.setPosition((double) shootingAngle /180); //Scale Angle from 0-180 to 0-1

            //Intake Motor
            double intakePower = 0;
            if(gamepad1.b){
                intakePower = -0.65;
            }

            else if (gamepad1.right_trigger > 0.1) {
                intakePower = 0.55;  // ดูดเร็ว (กด R2)
            }
            else{
                intakePower = 0.18;
            }
            M_AIN.setPower(intakePower);

            //Spin-Up
            M_S0.setPower(gamepad2.a ? 1:0.25); //A pressed = spin up
            M_S0.setPower(gamepad2.x ? -1:M_S0.getPower()); //X pressed = reverse direction with maximum speed

            //Blocking motor & shooting motor
            if(gamepad2.b){
                telemetry.addLine("Shooting!");
                M_S1.setPower(-1);
                if(gamepad2.bWasPressed()){
                    motorTimer.schedule(motorAct, 300); //Run this task once within 300 ms
                }

            }
            else{
                M_bl.setPower(0);
                M_S1.setPower(-0.35);
                if(gamepad2.bWasReleased()){
                    motorTimer.cancel();
                }
                motorTimer.cancel();
            }

            int pos = M_S1.getCurrentPosition();
            long now = System.currentTimeMillis();

            double dt = (now - lastTime) / 1000.0;     // วินาที
            int dPos = pos - lastPos;

            double ticksPerSec = dPos / dt;            // ความเร็วจริง
            double rpm = (ticksPerSec / 537.7) * 60;   // แปลงเป็น RPM

            telemetry.addData("M_AIN", M_AIN.getPower());
            telemetry.addData("M_S0", M_S0.getPower());
            telemetry.addData("M_bl", M_bl.getPower());
            telemetry.addData("M_S1", M_S1.getPower());
            telemetry.addData("SVR_L0", SVR_L0.getPosition());
            telemetry.addData("M_S1 speed (RPM)", rpm);
            telemetry.update();

        }
    }
}
