package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TeleOp: Control", group = "TeleOp")
public class TeleOpControlFinal extends LinearOpMode {

    DcMotor M_AIN, M_S0, M_S1, M_bl, M_LF, M_RF, M_LR, M_RR;
    Servo SVR_L0, SVR_L1, SVR_sw;
    //System Timer

    double speedMultiplier = 0.40;

    private double shootingPower1 = 0.46; //Initial shooting angle
    private double shootingPower2 = 0;
    private double angleAdd = 0.1; //Servo angle addition value
    int timerState = 0;
    int createTimer = 0;

    Thread shootingAct = new Thread(new Runnable() {
        @Override
        public void run(){

            if(gamepad2.b){
                telemetry.addLine("Shooting!");
                M_S1.setPower(-0.80);
                M_S0.setPower(1);
                try{
                    Thread.sleep(600);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                M_bl.setPower(-1);



            }
            else{
                M_bl.setPower(0);
                M_S1.setPower(-0.35);

            }
        }

    });


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
        SVR_sw = hardwareMap.get(Servo.class, "SVR_sw");

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
        SVR_L0.setPosition(0.46);
        SVR_L1.setPosition(0);
        SVR_sw.setPosition(0.1);
        waitForStart();

        while (opModeIsActive()){

            if(gamepad2.right_bumper){
                shootingPower1 = (Math.max(0.18, Math.min(0.46, SVR_L0.getPosition())) - 0.001); //Scale Angle from 0-180 to 0-1
                shootingPower2 = (Math.max(0, Math.min(0.28, SVR_L1.getPosition())) + (0.001));

            }
            if(gamepad2.left_bumper){
                shootingPower1 = (Math.max(0.18, Math.min(0.46, SVR_L0.getPosition())) + 0.001); //Scale Angle from 0-180 to 0-1
                shootingPower2 = (Math.max(0, Math.min(0.28, SVR_L1.getPosition())) - (0.001));
            }
            if(gamepad2.y){
                SVR_sw.setPosition(0.5);
            }
            else{
                SVR_sw.setPosition(0.1);
            }
            SVR_L0.setPosition(shootingPower1);
            SVR_L1.setPosition(shootingPower2);





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
            M_S0.setPower(gamepad2.a ? 1:0); //A pressed = spin up
            M_S0.setPower(gamepad2.x ? -1:M_S0.getPower()); //X pressed = reverse direction with maximum speed

            //Blocking motor & shooting motor
            shootingAct.start();

            //Driving
            double forwardBackward = -gamepad1.left_stick_y; //L3 เดินหน้าเดินหลัง

            double strafe = gamepad1.left_stick_x;           //L3 สไลด์ซ้ายขวา

            double rotate = gamepad1.right_stick_x;          //R3 หันทางซ้ายหันทางขวา

            //ระบบคำนวณกำลังล้อ Mecanum 4 ล้อ
            if(gamepad1.left_stick_y > -0.001 && gamepad1.left_stick_y < 0.001){
                speedMultiplier = 0.35;
            }
            else{
                speedMultiplier = 0.5;
            }

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

            if(gamepad1.left_stick_x > -0.001 && gamepad1.left_stick_x < 0.001 && gamepad1.left_stick_y < 0.001 && gamepad1.left_stick_y > -0.001 && gamepad1.right_stick_x > -0.001 && gamepad1.right_stick_x < 0.001){
                M_RF.setPower(0);
            }
            else if(gamepad1.left_stick_y < 0.001 && gamepad1.left_stick_y > -0.001){
                M_RF.setPower(powerRF);
            }
            else{
                M_RF.setPower(powerRF+0.07);
            }
            M_LR.setPower(powerLR);
            M_RR.setPower(powerRR);
            telemetry.addData("M_AIN", M_AIN.getPower());
            telemetry.addData("M_S0", M_S0.getPower());
            telemetry.addData("M_bl", M_bl.getPower());
            telemetry.addData("M_S1", M_S1.getPower());

            telemetry.addData("SVR_L0", SVR_L0.getPosition());
            telemetry.addData("SVR_L1", SVR_L1.getPosition());
            telemetry.addData("SVR_sw", SVR_sw.getPosition());
            telemetry.addLine("--Driving Section--");
            telemetry.addData("Power LF", M_LF.getPower());
            telemetry.addData("Power RF", M_RF.getPower());
            telemetry.addData("Power LR", M_LR.getPower());
            telemetry.addData("Power RR", M_RR.getPower());
            telemetry.update();

        }
    }
}
