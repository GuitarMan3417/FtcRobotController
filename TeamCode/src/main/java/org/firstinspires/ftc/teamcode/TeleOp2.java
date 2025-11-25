package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp2", group = "TeleOp1")
public class TeleOp2 extends LinearOpMode {

    // ==============================
    //   ประกาศตัวแปรอุปกรณ์ทั้งหมด
    // ==============================

    DcMotor M_LF, M_RF, M_LR, M_RR;   // มอเตอร์ล้อทั้ง 4 (Mecanum)
    DcMotor M_AIN;                    // มอเตอร์ดูดบอล
    DcMotor M_S0, M_S1;               // มอเตอร์ส่วนกลไกยิง/ดัน
    Servo SVR_L0;                     // เซอร์โวที่ 1
    Servo SVR_L1;                     // เซอร์โวที่ 2

    @Override
    public void runOpMode() {

        // ====================================
        //   ดึงอุปกรณ์จาก Hardware Config
        // ====================================

        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");

        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");
        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");

        // ========================================================
        //   ตั้งทิศทางมอเตอร์ (สำคัญมากสำหรับ Mecanum ให้วิ่งตรง)
        // ========================================================

        M_LF.setDirection(DcMotorSimple.Direction.REVERSE);  // ล้อซ้ายหน้า
        M_LR.setDirection(DcMotorSimple.Direction.REVERSE);  // ล้อซ้ายหลัง
        M_RF.setDirection(DcMotorSimple.Direction.FORWARD);  // ล้อขวาหน้า กลับทิศ
        M_RR.setDirection(DcMotorSimple.Direction.FORWARD);  // ล้อขวาหลัง กลับทิศ

        // ====================================
        //   ทำให้มอเตอร์เบรกเมื่อปล่อยคันโยก
        // ====================================

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ========================================================
        //   ค่าเริ่มต้น Servo (แปลงองศา -> ค่าระหว่าง 0.0 - 1.0)
        // ========================================================

        double servoMin = 0.0;
        double servoMax = 160.0;
        double minPos = servoMin / 180.0;
        double maxPos = servoMax / 180.0;
        double currentPos = minPos;  // เริ่มที่ตำแหน่งต่ำสุด
        SVR_L0.setPosition(currentPos);

        double servo1Min = 0.0;
        double servo1Max = 180.0;
        double servo1Pos = servo1Min / 180.0; // เริ่มต้น 0°
        SVR_L1.setPosition(servo1Pos);

        // รอจนผู้ใช้กด START
        waitForStart();

        // ใช้คูณความเร็ว (1.0 = เต็ม)
        double speedMultiplier = 1.0;

        while (opModeIsActive()) {

            // ========================================================
            //   ⭐⭐⭐ ระบบขับเคลื่อน — เดินหน้า/ถอยหลังใช้ L3 ⭐⭐⭐
            // ========================================================

            // เดินหน้า/ถอยหลัง = L3 ขึ้น/ลง
            // หมายเหตุ: gamepad1.left_stick_y ขึ้น = -1 จึงต้องใส่ - หน้า
            double forwardBackward = -gamepad1.left_stick_y;

            // สไลด์ซ้าย-ขวา = L3 ซ้าย/ขวา
            double strafe = gamepad1.left_stick_x;

            // หมุนซ้าย-ขวา = R3 ซ้าย/ขวา
            double rotate = gamepad1.right_stick_x;

            // สูตรคำนวณกำลังล้อ Mecanum 4 ล้อ
            double powerLF = (forwardBackward + strafe + rotate) * speedMultiplier;
            double powerRF = (forwardBackward - strafe - rotate) * speedMultiplier;
            double powerLR = (forwardBackward - strafe + rotate) * speedMultiplier;
            double powerRR = (forwardBackward + strafe - rotate) * speedMultiplier;

            // ป้องกันค่ากำลังเกิน 1.0 (Normalize)
            double max = Math.max(1.0,
                    Math.max(Math.abs(powerLF),
                            Math.max(Math.abs(powerRF),
                                    Math.max(Math.abs(powerLR), Math.abs(powerRR)))));

            powerLF /= max;
            powerRF /= max;
            powerLR /= max;
            powerRR /= max;

            // ส่งกำลังไปมอเตอร์
            M_LF.setPower(powerLF);
            M_RF.setPower(powerRF);
            M_LR.setPower(powerLR);
            M_RR.setPower(powerRR);

            // ========================================================
            //   ระบบดูดบอล (Intake) — ใช้ปุ่ม gamepad1
            // ========================================================

            double intakePower = 0.0;

            if (gamepad1.b) {
                intakePower = -0.60; // ปล่อยลูกบอลออก
            }
            else if (gamepad1.left_trigger > 0.1) {
                intakePower = 0.18;  // ดูดช้า (กด L2)
            }
            else if (gamepad1.right_trigger > 0.1) {
                intakePower = 0.50;  // ดูดเร็ว (กด R2)
            }

            M_AIN.setPower(intakePower);

            // ========================================================
            //   กลไกมอเตอร์ M_S0 — ปุ่ม A (gamepad2)
            // ========================================================

            M_S0.setPower(gamepad2.a ? 0.8 : 0.0);

            // ========================================================
            //   กลไกยิง/ดันลูกด้วย M_S1 + Servo SVR_L0
            //   ปุ่ม B บน gamepad2
            // ========================================================

            if (gamepad2.b) {
                M_S1.setPower(-1.0);   // หมุนแรงเวลาเตรียมยิง
                sleep(300);            // รอให้สปีดนิ่งก่อน
                currentPos = maxPos;   // ดันเซอร์โวออก
            } else {
                M_S1.setPower(-0.5);   // หมุนเบาๆ ตลอดเวลา
                currentPos = minPos;   // เซอร์โวกลับตำแหน่งเดิม
            }

            SVR_L0.setPosition(currentPos);

            // ========================================================
            //   Servo ตัวที่ 2 (SVR_L1)
            //   ใช้ L2 ลดองศา, R2 เพิ่มองศา (gamepad2)
            // ========================================================

            double servo1Step = 0.01;  // ความไวในการปรับ (ยิ่งเล็กยิ่งละเอียด)

            if (gamepad2.left_trigger > 0.1) servo1Pos -= servo1Step;
            if (gamepad2.right_trigger > 0.1) servo1Pos += servo1Step;

            // ล็อกไม่ให้เกิน 0° - 180°
            servo1Pos = Math.max(servo1Min / 180.0,
                    Math.min(servo1Max / 180.0, servo1Pos));

            SVR_L1.setPosition(servo1Pos);

            // ========================================================
            //   แสดงข้อมูลบนหน้าจอ Driver Hub
            // ========================================================

            telemetry.addLine("=== 24552 KhonNex ===");
            telemetry.addData("LF/RF/LR/RR", "%.2f  %.2f  %.2f  %.2f",
                    powerLF, powerRF, powerLR, powerRR);
            telemetry.addData("Servo L0", "%.0f°", currentPos * 180.0);
            telemetry.addData("Servo L1", "%.0f°", servo1Pos * 180.0);
            telemetry.update();

            sleep(20); // ลดการกระตุก
        }
    }
}
