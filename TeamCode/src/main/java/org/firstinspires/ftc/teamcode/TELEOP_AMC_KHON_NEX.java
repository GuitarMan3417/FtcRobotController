package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.PublicKey;

@TeleOp(name = "TELEOP_AMC_KHON_NEX", group = "TeleOp")
public class TELEOP_AMC_KHON_NEX extends LinearOpMode {

    // ==================================
    //   ประกาศตัวแปรทั้งหมด / Hardware All
    // ==================================

    DcMotor M_LF, M_RF, M_LR, M_RR; //Motor ขับเคลื่อนที่ 4 ล้อ ( Mecanum )

    DcMotor M_AIN;                   //Motor ดึงบอลเข้า

    DcMotor M_S0, M_S1, M_bl;              //Motor ดันบอลและยิงบอล

    Servo SVR_L0, SVR_L1;            //Servo 2 ตัว

    @Override
    public void runOpMode() {

        // ==================================
        //   ชื่อตัวแปรทั้งหมด Hardware Config
        // ==================================

        M_LF = hardwareMap.get(DcMotor.class, "M_LF");       //Motor ล้อซ้ายบน
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");       //Motor ล้อขวาบน
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");       //Motor ล้อซ้ายล่าง
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");       //Motor ล้อขวาล่าง

        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");     //Motor ดึงบอลเข้า

        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");       //Motor ดันบอล
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");       //Motor ยิงบอล
        M_bl = hardwareMap.get(DcMotor.class, "M_bl");       //Motor ยิงบอล



        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");     //Servo ดันบอลเข้ายิง
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");     //Servo ปรับองศาการยิง

        // ==================================
        //   ตั้งทิศทางของมอเตอร์สำหรับ Mecanum
        // ==================================

        M_LF.setDirection(DcMotorSimple.Direction.FORWARD); // ล้อซ้ายบน
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD); // ล้อขวาบน
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE); // ล้อซ้ายล่าง
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE); // ล้อขวาล่าง

        // ==================================
        //   ระบบเบรกเมื่อปล่อยคันโยก GamePad
        // ==================================

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==================================
        //   Servo Hardware Config
        // ==================================

        double servo1Min = 0.0;
        double servo1Max = 180.0;
        double servo1Pos = servo1Min / 180.0; // เริ่มต้น 0°
        SVR_L1.setPosition(servo1Pos);

        waitForStart(); //Start Code

        double speedMultiplier = 0.5;  //   ความเร็ว ( 0.5 = เต็ม )

        while (opModeIsActive()) {

            double forwardBackward = -gamepad1.left_stick_y; //L3 เดินหน้าเดินหลัง

            double strafe = gamepad1.left_stick_x;           //L3 สไลด์ซ้ายขวา

            double rotate = gamepad1.right_stick_x;          //R3 หันทางซ้ายหันทางขวา

            //ระบบคำนวณกำลังล้อ Mecanum 4 ล้อ
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

            // ==================================
            //   P1 ระบบดึงบอลเข้า
            // ==================================

            double intakePower = 0.0;
            if (gamepad1.b) {
                intakePower = -0.60;                // ปล่อย Artifact ออก B
            } else if (gamepad1.left_trigger > 0.1) {
                intakePower = 0.18;                     // ดึง Artifact เข้าด้วยความเร็ว 0.18 L2
            } else if (gamepad1.right_trigger > 0.1) {
                intakePower = 0.50;                     // ดึง Artifact เข้าด้วยความเร็ว 0.50 R2
            }
            M_AIN.setPower(intakePower);

            // ==================================
            //   P2 ระบบยิงบอล A ทำงานตามระบบ
            // ==================================

            long delayStartTime = 0;
            boolean isButtonAPressed = false;
            boolean hasMotorStarted = false;
            long delayMillis = 300; // 0.3 วินาที

            // ใน loop
            if (gamepad2.a && !isButtonAPressed) {
                M_S1.setPower(-1.0);
                delayStartTime = System.currentTimeMillis();  // เก็บเวลาเริ่มต้น
                isButtonAPressed = true;
                hasMotorStarted = false;
            }

            if (isButtonAPressed && !hasMotorStarted) {
                long elapsed = System.currentTimeMillis() - delayStartTime;
                telemetry.addData("Delay Progress", "%d / %d ms", elapsed, delayMillis);
                telemetry.update();

                if (elapsed >= delayMillis) {
                    M_S0.setPower(1.0);
                    M_bl.setPower(-1.0);
                    hasMotorStarted = true;
                }
            }

            if (!gamepad2.a) {
                isButtonAPressed = false;
                hasMotorStarted = false;
                M_S1.setPower(-0.5);
                M_S0.setPower(0.0);
                M_bl.setPower(0.0);
            }


            // ==================================
            // Driver Hub KhonNex
            // ==================================

            telemetry.addLine("24552 AMC KHONE NEX ");

            // แสดงค่ากำลังล้อ Mecanum
            telemetry.addData("LF Power", "%.2f", powerLF);
            telemetry.addData("RF Power", "%.2f", powerRF);
            telemetry.addData("LR Power", "%.2f", powerLR);
            telemetry.addData("RR Power", "%.2f", powerRR);
            // แสดงสถานะระบบดึงบอล
            telemetry.addData("Intake Power", "%.2f", intakePower);
            // แสดงสถานะระบบยิงบอล
            telemetry.addData("M_S1 Power", "%.2f", M_S1.getPower());
            telemetry.addData("M_S0 Power", "%.2f", M_S0.getPower());
            telemetry.addData("M_bl Power", "%.2f", M_bl.getPower());
            // แสดงสถานะปุ่ม A และ delay
            telemetry.addData("Button A Pressed", isButtonAPressed);
            if (isButtonAPressed) {
                double elapsed = (getRuntime() * 1000) - delayStartTime;
                telemetry.addData("M_S1 Delay Elapsed (ms)", "%.0f / %.0f", elapsed, delayMillis);
            }
            // แสดงค่า Servo
            telemetry.addData("Servo L1 Pos", "%.2f°", servo1Pos * 180.0);
            // อัพเดต telemetry
            telemetry.update();
            sleep(20); //ลดการกินCPU
        }
    }
}
