package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpOp", group = "TeleOp")
public class TeleOpOp extends LinearOpMode {

    // --- มอเตอร์ขับเคลื่อน Mecanum ---
    DcMotor M_LF, M_RF, M_LR, M_RR;

    // --- มอเตอร์ gamepad1 ดูดบอล ---
    DcMotor M_AIN;

    // --- มอเตอร์ gamepad2 ---
    DcMotor M_S0, M_S1;

    // --- Servo Smart Servo ---
    Servo servo0;

    @Override
    public void runOpMode() {

        // --- Map มอเตอร์ขับเคลื่อน ---
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        // --- Map มอเตอร์ดูดบอล (gamepad1) ---
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");

        // --- Map มอเตอร์ gamepad2 ---
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");

        // --- Map Servo ---
        servo0 = hardwareMap.get(Servo.class, "servo0");

        // --- ตั้งทิศทางมอเตอร์ ---
        M_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);
        //M_S1.setDirection(DcMotor.Direction.REVERSE); // M_S1 หมุนตรงข้าม M_S0

        // --- เปิดเบรกอัตโนมัติทุกมอเตอร์ ---
        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- ตั้งค่า Servo ---
        double servoMin = 0.0;
        double servoMax = 160.0;
        double minPos = servoMin / 180.0;
        double maxPos = servoMax / 180.0;
        double targetPos = minPos;
        double currentPos = minPos;
        double step = 0.01;
        servo0.setPosition(currentPos);

        waitForStart();

        double speedMultiplier = 1.0;

        while (opModeIsActive()) {

            // --- ควบคุมการขับเคลื่อน (gamepad1) ---
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
                intakePower = 0.60;   // ปล่อยบอลออก
            } else if (gamepad1.left_bumper) {
                intakePower = -0.13;    // ดูดช้า
            } else if (gamepad1.right_bumper) {
                intakePower = -0.50;    // ดูดเร็ว
            }
            M_AIN.setPower(intakePower);

            // --- มอเตอร์ดูดบอล (M_S0, M_S1) gamepad2 ---
            double powerM_S1 = gamepad2.a ? 1.0 : 0.1;
            M_S0.setPower(powerM_S1);
            M_S1.setPower(powerM_S1);

            // --- ควบคุม Servo ด้วย L2/R2 ของ Gamepad2 ---
            if (gamepad2.left_trigger > 0.5) {
                targetPos = maxPos;  // ไป 160° (ประมาณ 180°)
            } else if (gamepad2.right_trigger > 0.5) {
                targetPos = minPos;  // กลับ 0° (ประมาณ 30°)
            }

            if (Math.abs(currentPos - targetPos) > 0.005) {
                if (currentPos < targetPos)
                    currentPos += step;
                else
                    currentPos -= step;

                currentPos = Math.max(minPos, Math.min(maxPos, currentPos));
                servo0.setPosition(currentPos);
            }

            double currentAngle = currentPos * 180.0;
            double targetAngle = targetPos * 180.0;

            telemetry.addLine("=== 24552 KhonNex ===");
            telemetry.addData("Speed", "%.2f", speedMultiplier);
            telemetry.addData("Servo Target", "%.0f°", targetAngle);
            telemetry.addData("Servo Current", "%.0f°", currentAngle);
            telemetry.addData("Intake (M_AIN)", "%.2f", M_AIN.getPower());
            telemetry.addData("L3", "%.2f", strafe);
            telemetry.addData("L2/R2", "%.2f/%.2f", forward, backward);
            telemetry.addData("LF/RF/LR/RR", "%.2f/%.2f/%.2f/%.2f",
                    powerLF, powerRF, powerLR, powerRR);
            telemetry.addData("M_S0/2 (GP2)", "%.2f/%.2f",
                    M_S0.getPower(), M_S1.getPower());
            telemetry.update();

            sleep(20);
        }
    }
}
