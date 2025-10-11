package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LinearOp-Team", group="TeleOp")
public class TeleOpGPT extends LinearOpMode {

    private DcMotor M_LF; // มอเตอร์ซ้ายหน้า
    private DcMotor M_RF; // มอเตอร์ขวาหน้า
    private DcMotor M_LR; // มอเตอร์ซ้ายหลัง
    private DcMotor M_RR; // มอเตอร์ขวาหลัง

    @Override
    public void runOpMode() {

        // --- เชื่อมต่อมอเตอร์กับชื่อใน Hardware Configuration ---
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        // --- ตั้งทิศทางหมุนให้ตรงกัน ---
        M_LF.setDirection(DcMotor.Direction.REVERSE);
        M_LR.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("System Ready!");
        telemetry.update();

        waitForStart(); // รอจนผู้ใช้กดปุ่ม Start

        // --- ลูปควบคุมหลัก ---
        while (opModeIsActive()) {

            // --- รับค่าจากจอย ---
            double drive = -(gamepad1.right_trigger - gamepad1.left_trigger);  // เดินหน้า/ถอยหลัง
            double strafe = gamepad1.left_stick_x;  // เลื่อนข้าง
            double turn = gamepad1.right_stick_x;   // หมุนซ้าย/ขวา

            // --- สูตรควบคุมแบบ Mecanum ---
            double powerLF = drive + strafe + turn;
            double powerRF = drive - strafe - turn;
            double powerLR = drive - strafe + turn;
            double powerRR = drive + strafe - turn;

            // --- ปรับให้ค่าไม่เกิน 1 ---
            double max = Math.max(1.0,
                    Math.max(Math.abs(powerLF),
                            Math.max(Math.abs(powerRF),
                                    Math.max(Math.abs(powerLR), Math.abs(powerRR)))));
            powerLF /= max;
            powerRF /= max;
            powerLR /= max;
            powerRR /= max;

            // --- ส่งกำลังไปมอเตอร์ ---
            M_LF.setPower(powerLF);
            M_RF.setPower(powerRF);
            M_LR.setPower(powerLR);
            M_RR.setPower(powerRR);

            // --- แสดงค่าทางจอ (Debug) ---
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.addData("LF", M_LF.getPower());
            telemetry.addData("RF", M_RF.getPower());
            telemetry.addData("LR", M_LR.getPower());
            telemetry.addData("RR", M_RR.getPower());
            telemetry.update();
        }
    }
}