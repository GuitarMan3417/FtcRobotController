package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpServo", group = "TeleOp")
public class TeleOpServo extends LinearOpMode {

    // --- มอเตอร์ขับเคลื่อน Mecanum ---
    DcMotor M_LF, M_RF, M_LR, M_RR;

    // --- มอเตอร์ดูดบอล ---
    DcMotor intakeMotor; // port 0

    // --- มอเตอร์ gamepad2 ---
    DcMotor motor1, motor2;

    // --- Servo Smart Servo ---
    Servo servo0;

    @Override
    public void runOpMode() {

        // --- Map มอเตอร์ขับเคลื่อน ---
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        // --- Map มอเตอร์ดูดบอล ---
        intakeMotor = hardwareMap.get(DcMotor.class, "motor0");

        // --- Map มอเตอร์ gamepad2 ---
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        // --- Map Servo ---
        servo0 = hardwareMap.get(Servo.class, "servo0");

        // --- ตั้งทิศทางมอเตอร์ ---
        M_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE); // motor2 หมุนตรงข้าม motor1

        // --- เปิดเบรกอัตโนมัติทุกมอเตอร์ ---
        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- ตั้งค่า Servo ---
        double servoMin = 30.0;
        double servoMax = 180.0;
        double minPos = servoMin / 180.0;
        double maxPos = servoMax / 180.0;
        double targetPos = minPos;
        double currentPos = minPos;
        double step = 0.01;  // ความเร็วในการเคลื่อนที่ servo
        servo0.setPosition(currentPos);

        waitForStart();

        double speedMultiplier = 1;
        boolean reversed = false;

        // --- ให้มอเตอร์ดูดลูกบอลทำงานเบาๆ ตั้งแต่เริ่ม ---
        intakeMotor.setPower(0.15);

        while (opModeIsActive()) {

            // --- ปรับความเร็ว Mecanum ---
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

            // --- จำกัดค่า ---
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

            // --- ควบคุมมอเตอร์ gamepad2 ---
            double powerMotor2 = gamepad2.a ? 1.0 : 0.1; // กด A → full, ไม่กด → idle
            motor1.setPower(powerMotor2);
            motor2.setPower(powerMotor2);

            // --- ควบคุม Servo ด้วย L2/R2 ของ Gamepad2 ---
            if (gamepad2.left_trigger > 0.5) {
                targetPos = maxPos;  // ไป 180°
            } else if (gamepad2.right_trigger > 0.5) {
                targetPos = minPos;  // กลับ 30°
            }

            // ค่อย ๆ เคลื่อน servo
            if (Math.abs(currentPos - targetPos) > 0.005) {
                if (currentPos < targetPos)
                    currentPos += step;
                else
                    currentPos -= step;

                currentPos = Math.max(minPos, Math.min(maxPos, currentPos));
                servo0.setPosition(currentPos);
            }

            // --- แปลงตำแหน่ง servo เป็นองศา ---
            double currentAngle = currentPos * 180.0;
            double targetAngle = targetPos * 180.0;

            // --- แสดงสถานะบนหน้าจอ ---
            telemetry.addLine("=== 24552 KhonNex ===");
            telemetry.addData("Speed", "%.2f", speedMultiplier);
            telemetry.addData("Direction", reversed ? "REVERSED (Y)" : "NORMAL");
            telemetry.addData("Intake", gamepad1.a ? "FULL (1.0)" : "IDLE (0.15)");
            telemetry.addData("Servo Target", "%.0f°", targetAngle);
            telemetry.addData("Servo Current", "%.0f°", currentAngle);
            telemetry.addData("L3", "%.2f", strafe);
            telemetry.addData("L2/R2", "%.2f/%.2f", forward, backward);
            telemetry.addData("LF/RF/LR/RR", "%.2f/%.2f/%.2f/%.2f",
                    powerLF, powerRF, powerLR, powerRR);
            telemetry.addData("Motor1/2 (GP2)", "%.2f/%.2f",
                    motor1.getPower(), motor2.getPower());
            telemetry.update();

            sleep(20);
        }
    }
}
