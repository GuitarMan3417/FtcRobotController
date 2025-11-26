package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpManualControl", group = "TeleOp")
public class ManualControl extends LinearOpMode {

    // ==============================
    //   ประกาศตัวแปรอุปกรณ์ทั้งหมด
    // ==============================

    DcMotor M_LF, M_RF, M_LR, M_RR;   // มอเตอร์ล้อทั้ง 4 (Mecanum)

    @Override
    public void runOpMode() {

        // ====================================
        //   ดึงอุปกรณ์จาก Hardware Config
        // ====================================

        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");


        // ========================================================
        //   ตั้งทิศทางมอเตอร์ (สำคัญมากสำหรับ Mecanum ให้วิ่งตรง)
        // ========================================================

        M_LF.setDirection(DcMotorSimple.Direction.FORWARD);  // ล้อซ้ายหน้า
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);  // ล้อซ้ายหลัง
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);  // ล้อขวาหน้า กลับทิศ
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);  // ล้อขวาหลัง กลับทิศ

        // ====================================
        //   ทำให้มอเตอร์เบรกเมื่อปล่อยคันโยก
        // ====================================

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        // ใช้คูณความเร็ว (1.0 = เต็ม)
        double speedMultiplier = 0.5;

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


            telemetry.addLine("=== 24552 KhonNex ===");
            telemetry.addData("LF/RF/LR/RR", "%.2f  %.2f  %.2f  %.2f",
                    powerLF, powerRF, powerLR, powerRR);
            telemetry.addData("Speed Multiplier", "%.2f", speedMultiplier);
            telemetry.addData("Joystick FB/Strafe/Rotate", "%.2f / %.2f / %.2f",
                    forwardBackward, strafe, rotate);
            telemetry.addData("Joystick Raw", "Left Y: %.2f Left X: %.2f Right X: %.2f",
                    gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.update();


            sleep(20); // ลดการกระตุก
        }
    }
}
