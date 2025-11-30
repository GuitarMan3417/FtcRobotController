package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "TELEOP_AMC_KHON_NEX", group = "TeleOp")
public class TELEOP_AMC_KHON_NEX extends LinearOpMode {

    DcMotor M_LF, M_RF, M_LR, M_RR;   // Mecanum drive
    DcMotor M_AIN;                   // Intake motor
    DcMotor M_S0, M_S1, M_bl;        // Shooting motors
    Servo SVR_L0, SVR_L1;            // Servos

    @Override
    public void runOpMode() {

        // ===============================
        // Hardware Map
        // ===============================
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");

        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");
        M_bl = hardwareMap.get(DcMotor.class, "M_bl");

        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");

        // ===============================
        // Motor Directions
        // ===============================
        M_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===============================
        // Zero Power Behavior
        // ===============================
        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===============================
        // Servo init
        // ===============================
        double servo1Min = 0.0;
        double servo1Max = 180.0;
        double servo1Pos = servo1Min / 180.0; // เริ่มต้น 0°
        SVR_L1.setPosition(servo1Pos);

        waitForStart();

        double speedMultiplier = 0.5;

        // Shooting System Vars
        ElapsedTime shooterTimer = new ElapsedTime();
        boolean isButtonAPressed = false;
        boolean hasMotorStarted = false;
        double delayMillis = 500; // 0.3 second

        while (opModeIsActive()) {

            // ===============================
            // Mecanum Drive
            // ===============================
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double powerLF = (forward + strafe + rotate) * speedMultiplier;
            double powerRF = (forward - strafe - rotate) * speedMultiplier;
            double powerLR = (forward - strafe + rotate) * speedMultiplier;
            double powerRR = (forward + strafe - rotate) * speedMultiplier;

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


            // ===============================
            // Intake System
            // ===============================
            double intakePower = 0.18;

            if (gamepad1.b) {
                intakePower = -0.60; // reverse
            } else if (gamepad1.left_trigger > 0.1) {
                intakePower = 0.18;  // slow intake
            } else if (gamepad1.right_trigger > 0.1) {
                intakePower = 0.50;  // fast intake
            }

            M_AIN.setPower(intakePower);


            // ===============================
            // Spin-up Motor (A button)
            // ===============================
            if (gamepad2.a) {
                M_S0.setPower(1.0);}

            // ===============================
            // Shooting Motor (B button)
            // ===============================

            // Press B → start wind-up
            if (gamepad2.b && !isButtonAPressed) {
                M_S1.setPower(-1.0);  // wind-up
                shooterTimer.reset();
                isButtonAPressed = true;
                hasMotorStarted = false;
            }

            // After delay → shoot
            if (isButtonAPressed && !hasMotorStarted) {
                if (shooterTimer.milliseconds() >= delayMillis) {
                    M_S0.setPower(1.0);   // feed
                    M_bl.setPower(-1.0);  // shoot
                    hasMotorStarted = true;
                }
            }

            // Release B → stop
            if (!gamepad2.b) {
                M_S1.setPower(-0.5);  // slow wind
                M_S0.setPower(0);
                M_bl.setPower(0);
                isButtonAPressed = false;
                hasMotorStarted = false;
            }

            double servo1Step = 0.01;  // ความไวในการปรับ (ยิ่งเล็กยิ่งละเอียด)

            if (gamepad2.left_trigger > 0.1) servo1Pos -= servo1Step;
            if (gamepad2.right_trigger > 0.1) servo1Pos += servo1Step;

            // ล็อกไม่ให้เกิน 0° - 180°
            servo1Pos = Math.max(servo1Min / 180.0,
                    Math.min(servo1Max / 180.0, servo1Pos));

            SVR_L1.setPosition(servo1Pos);


            // ===============================
            telemetry.addData("LF", powerLF);
            telemetry.addData("RF", powerRF);
            telemetry.addData("LR", powerLR);
            telemetry.addData("RR", powerRR);

            telemetry.addData("Intake", intakePower);
            telemetry.addData("Shoot Delay", shooterTimer.milliseconds());
            telemetry.update();

            sleep(20);
        }
    }
}
