package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;
@TeleOp(name="ExperimentalOp", group = "TeleOp")
public class TeleOpManualExperiment extends LinearOpMode {

    DcMotor M_AIN, M_S0, M_S1, M_bl, M_LF, M_RF, M_LR, M_RR;
    Servo SVR_L0, SVR_L1;
    //System Timer

    TimerTask motorAct = new TimerTask() {
        @Override
        public void run() {
            M_bl.setPower(1);
        }
    };

    private int shootingAngle = 0; //Initial shooting angle
    private int angleAdd = 1; //Servo angle addition value
    int timerState = 0;
    int createTimer = 0;
    long runtime = System.nanoTime();
    public Timer motorTimer = new Timer();
    public void shootingAct() throws InterruptedException {
        if(gamepad2.b){
            telemetry.addLine("Shooting!");
            M_S1.setPower(-1);
            M_S0.setPower(1);
            Thread.sleep(600);
            M_bl.setPower(-1);


        }
        else{
            M_bl.setPower(0);
            M_S1.setPower(-0.35);

        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
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

        // ==================================
        //   ตั้งทิศทางของมอเตอร์สำหรับ Mecanum
        // ==================================

        M_LF.setDirection(DcMotorSimple.Direction.FORWARD); // ล้อซ้ายบน
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD); // ล้อขวาบน
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE); // ล้อซ้ายล่าง
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE); // ล้อขวาล่าง

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        M_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()){
            if(createTimer == 1){
                Timer motorTimer = new Timer();
                createTimer = 2;
            }
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
            shootingAct();

            //Driving
            double forwardBackward = -gamepad1.left_stick_y; //L3 เดินหน้าเดินหลัง

            double strafe = gamepad1.left_stick_x;           //L3 สไลด์ซ้ายขวา

            double rotate = gamepad1.right_stick_x;          //R3 หันทางซ้ายหันทางขวา

            //ระบบคำนวณกำลังล้อ Mecanum 4 ล้อ
            double speedMultiplier = 0.5;
            double powerLF = (forwardBackward + strafe + rotate) * speedMultiplier;
            double powerRF = (forwardBackward - strafe - rotate) * speedMultiplier;
            double powerLR = (forwardBackward - strafe + rotate) * speedMultiplier;
            double powerRR = (forwardBackward + strafe - rotate) * speedMultiplier;

            //ป้องกันค่ากำลังเกิน 1.0
            double max = Math.max(1.0,
                    Math.max(Math.abs(powerLF),
                            Math.max(Math.abs(powerRF),
                                    Math.max(Math.abs(powerLR), Math.abs(powerRR)))));

            powerLF /= max;
            powerRF /= max;
            powerLR /= max;
            powerRR /= max;

            //ส่งกำลังไป Motor
            M_LF.setPower(powerLF);
            M_RF.setPower(powerRF);
            M_LR.setPower(powerLR);
            M_RR.setPower(powerRR);
            telemetry.addData("M_AIN", M_AIN.getPower());
            telemetry.addData("M_S0", M_S0.getPower());
            telemetry.addData("M_bl", M_bl.getPower());
            telemetry.addData("M_S1", M_S1.getPower());
            telemetry.addData("SVR_L0", SVR_L0.getPosition());
            telemetry.addData("Timer State", timerState);
            telemetry.addData("Create Timer", createTimer);
            telemetry.update();

        }
    }
}
