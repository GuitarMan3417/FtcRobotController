package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "TeleOpNew", group = "TeleOp")
public class TeleOpNew extends LinearOpMode {

    DcMotor M_LF, M_RF, M_LR, M_RR;
    DcMotor intakeMotor; // มอเตอร์ดูดบอล (port 0)

    @Override
    public void runOpMode() {

        // --- Map มอเตอร์ขับเคลื่อน ---
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        // --- Map มอเตอร์ดูดบอล (ต่อที่พอร์ต 0 ของ Expansion Hub) ---
            intakeMotor = hardwareMap.get(DcMotor.class, "motor0");

            // --- ตั้งทิศทางมอเตอร์ ---
            M_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- เปิดเบรกอัตโนมัติทุกมอเตอร์ ---
        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        double speedMultiplier = 1;
        boolean reversed = false;

        // --- ให้มอเตอร์ดูดลูกบอลทำงานเบาๆ ตั้งแต่เริ่ม ---
        intakeMotor.setPower(0.15);

        while (opModeIsActive()) {

            // --- ปรับความเร็ว ---
            if (gamepad1.left_bumper) { // L1 เพิ่มความเร็ว
                speedMultiplier += 0.05;
                if (speedMultiplier > 1.0) speedMultiplier = 1.0;
                sleep(150);
            }
            if (gamepad1.right_bumper) { // R1 ลดความเร็ว
                speedMultiplier -= 0.05;
                if (speedMultiplier < 0.2) speedMultiplier = 0.2;
                sleep(150);
            }

            // --- กลับหน้าหุ่น (Y) ---
            if (gamepad1.y) {
                reversed = !reversed;
                sleep(300);
            }

            // --- อ่านค่าจากจอย ---
            double forward = gamepad1.left_trigger;    // L2 เดินหน้า
            double backward = gamepad1.right_trigger;  // R2 ถอยหลัง
            double strafe = gamepad1.left_stick_x;     // L3 สไลด์
            double rotate = gamepad1.right_stick_x;    // R3 หมุน

            double drive = forward - backward;

            // --- กลับทิศทางหุ่น ---
            if (reversed) {
                drive *= -1;
                rotate *= -1;
                strafe *= -1;
            }

            // --- คำนวณกำลัง Mecanum ---
            double powerLF = (drive + strafe + rotate) * speedMultiplier;
            double powerRF = (drive - strafe - rotate) * speedMultiplier;
            double powerLR = (drive - strafe + rotate) * speedMultiplier;
            double powerRR = (drive + strafe - rotate) * speedMultiplier;

            // จำกัดค่า
            double max = Math.max(1.0,
                    Math.max(Math.abs(powerLF),
                            Math.max(Math.abs(powerRF),
                                    Math.max(Math.abs(powerLR), Math.abs(powerRR)))));
            powerLF /= max;
            powerRF /= max;
            powerLR /= max;
            powerRR /= max;

            // --- ส่งกำลังขับเคลื่อน ---
            M_LF.setPower(powerLF);
            M_RF.setPower(powerRF);
            M_LR.setPower(powerLR);
            M_RR.setPower(powerRR);

            // --- ควบคุมมอเตอร์ดูดบอล ---
            if (gamepad1.a) {
                intakeMotor.setPower(1.0);   // กด A → ดูดแรงเต็ม
            } else {
                intakeMotor.setPower(0.15);  // ไม่กด → หมุนเบาๆ ตลอด
            }

            // --- แสดงสถานะบนหน้าจอ ---
            telemetry.addLine("=== 24552 KhonNex ===");
            telemetry.addData("Speed", "%.2f", speedMultiplier);
            telemetry.addData("Direction", reversed ? "REVERSED (Y)" : "NORMAL");
            telemetry.addData("Intake", gamepad1.a ? "FULL (1.0)" : "IDLE (0.15)");
            telemetry.addData("L3", "%.2f", strafe);
            telemetry.addData("L2/R2", "%.2f/%.2f", forward, backward);
            telemetry.addData("LF/RF/LR/RR", "%.2f/%.2f/%.2f/%.2f", powerLF, powerRF, powerLR, powerRR);
            telemetry.update();
        }
    }
}
