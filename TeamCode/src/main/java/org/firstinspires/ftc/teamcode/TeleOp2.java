package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp2", group = "TeleOp1")

public class  TeleOp2 extends LinearOpMode {

    // --- มอเตอร์ขับเคลื่อน Mecanum ---
    DcMotor M_LF, M_RF, M_LR, M_RR;

    // --- มอเตอร์ดูดบอล (gamepad1) ---
    DcMotor M_AIN;

    // --- มอเตอร์และเซอร์โว (gamepad2) ---
    DcMotor M_S0, M_S1;
    Servo SVR_L0;

    @Override
    public void runOpMode() {

        // --- Map มอเตอร์ขับเคลื่อน ---
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        // --- Map มอเตอร์ดูดบอล (gamepad1) ---
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");

        // --- Map มอเตอร์และเซอร์โว (gamepad2) ---
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");
        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");

        // --- ตั้งทิศทางมอเตอร์ ---
        M_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- เปิดเบรกอัตโนมัติทุกมอเตอร์ ---
        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- ตั้งค่า Servo เริ่มต้น ---
        double servoMin = 0.0;
        double servoMax = 160.0;
        double minPos = servoMin / 180.0;
        double maxPos = servoMax / 180.0;
        double currentPos = minPos;
        SVR_L0.setPosition(currentPos);

        waitForStart();

        double speedMultiplier = 1.0;

        while (opModeIsActive()) {

            // --- ระบบขับเคลื่อน (gamepad1) ---
            double forward = gamepad1.left_trigger;    // L2 เดินหน้า
            double backward = gamepad1.right_trigger;  // R2 ถอยหลัง
            double strafe = gamepad1.left_stick_x;     // L3 สไลด์
            double rotate = gamepad1.right_stick_x;    // R3 หมุน
            double drive = forward - backward;

            double powerLF = (drive + strafe + rotate) * speedMultiplier;
            double powerRF = (drive - strafe - rotate) * speedMultiplier;
            double powerLR = (drive - strafe + rotate) * speedMultiplier;
            double powerRR = (drive + strafe - rotate) * speedMultiplier;

            double max = Math.max(1.0,
                    Math.max(Math.abs(powerLF),
                            Math.max(Math.abs(powerRF),
                                    Math.max(Math.abs(powerLR), Math.abs(powerRR)))));
            powerLF /= max;
            powerRF /= max;
            powerLR /= max;
            powerRR /= max;

            M_LF.setPower(powerLF);
            M_RF.setPower(powerRF);
            M_LR.setPower(powerLR);
            M_RR.setPower(powerRR);

            // --- มอเตอร์ดูดบอล (M_AIN) gamepad1 ---
            double intakePower = 0.0;
            if (gamepad1.b) {
                intakePower = -0.60;   // ปล่อยบอลออก
            } else if (gamepad1.left_bumper) {
                intakePower = 0.18;    // ดูดช้า
            } else if (gamepad1.right_bumper) {
                intakePower = 0.50;    // ดูดเร็ว
            }
            M_AIN.setPower(intakePower);

            // --- มอเตอร์ M_S0 (Gamepad2 ปุ่ม A) ---
            double powerM_S0 = gamepad2.a ? 0.8 : 0.0;
            M_S0.setPower(powerM_S0);

            // --- มอเตอร์ M_S1 + Servo SVR_L0 (Gamepad2 ปุ่ม B) ---
            if (gamepad2.b) {
                M_S1.setPower(-1.0);       // หมุนเต็มสปีดก่อน
                sleep(300);               // รอให้มอเตอร์หมุนก่อน 0.3 วินาที
                currentPos = maxPos;      // ดันบอลขึ้น
            } else {
                M_S1.setPower(-0.5);       // หมุนเบา ๆ ตลอดเวลา
                currentPos = minPos;      // Servo กลับตำแหน่งเริ่มต้น
            }

            SVR_L0.setPosition(currentPos);

            // --- แสดงสถานะ ---
            telemetry.addLine("=== 24552 KhonNex ===");
            telemetry.addData("Speed", "%.2f", speedMultiplier);
            telemetry.addData("Intake (M_AIN)", "%.2f", M_AIN.getPower());
            telemetry.addData("M_S0 Power", "%.2f", M_S0.getPower());
            telemetry.addData("M_S1 Power", "%.2f", M_S1.getPower());
            telemetry.addData("Servo SVR_L0", "%.0f°", currentPos * 180.0);
            telemetry.addData("L2/R2", "%.2f/%.2f", forward, backward);
            telemetry.addData("LF/RF/LR/RR", "%.2f/%.2f/%.2f/%.2f",
                    powerLF, powerRF, powerLR, powerRR);
            telemetry.update();

            sleep(20);
        }
    }
}
