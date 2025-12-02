package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp: Control111", group = "TeleOp")
public class TeleOpAllFunction extends LinearOpMode {

    DcMotor M_AIN, M_S0, M_S1, M_bl, M_LF, M_RF, M_LR, M_RR;
    Servo SVR_L0, SVR_L1, SVR_sw;

    // Servo Angle Control
    double angle = 0;        // เริ่มที่ 0°
    double angleMin = 0;
    double angleMax = 30;
    double servoStepDeg = 1; // เพิ่ม/ลดทีละ 1°

    @Override
    public void runOpMode() throws InterruptedException {

        // ================= Hardware Mapping =================
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
        SVR_sw = hardwareMap.get(Servo.class, "SVR_sw");

        // ================= Set Directions =================
        M_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);

        // ================= Zero Power Behavior =================
        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        M_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // ================= Intake Motor =================
            double intakePower = 0;
            if(gamepad1.b){
                intakePower = -0.65;
            } else if (gamepad1.right_trigger > 0.1) {
                intakePower = 0.55;
            } else {
                intakePower = 0.18;
            }
            M_AIN.setPower(intakePower);

            // ================= Spin-Up Motor =================
            M_S0.setPower(gamepad2.a ? 1 : 0);
            M_S0.setPower(gamepad2.x ? -1 : M_S0.getPower());

            // ================= Blocking & Shooting Motor =================
            if(gamepad2.b){
                M_S1.setPower(-1);
                M_S0.setPower(1); // Spin-up during shoot
                sleep(600);
                M_bl.setPower(-1);
            } else {
                M_bl.setPower(0);
                M_S1.setPower(-0.35);
            }

            // ================= Driving Control (Mecanum) =================
            double forwardBackward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double speedMultiplier = 0.40;
            double powerLF = (forwardBackward + strafe + rotate) * speedMultiplier;
            double powerRF = (forwardBackward - strafe - rotate) * speedMultiplier;
            double powerLR = (forwardBackward - strafe + rotate) * speedMultiplier;
            double powerRR = (forwardBackward + strafe - rotate) * speedMultiplier;

            double max = Math.max(1.0,
                    Math.max(Math.abs(powerLF),
                            Math.max(Math.abs(powerRF),
                                    Math.max(Math.abs(powerLR), Math.abs(powerRR)))));
            powerLF /= max;
            powerRF /= max;
            powerLR /= max;
            powerRR /= max;

            M_LF.setPower(powerLF);
            if(Math.abs(gamepad1.left_stick_x) < 0.001 && Math.abs(gamepad1.left_stick_y) < 0.001 && Math.abs(gamepad1.right_stick_x) < 0.001){
                M_RF.setPower(0);
            } else if(Math.abs(gamepad1.left_stick_y) < 0.001){
                M_RF.setPower(powerRF);
            } else {
                M_RF.setPower(powerRF+0.07);
            }
            M_LR.setPower(powerLR);
            M_RR.setPower(powerRR);

            // ================= Servo L0 & L1 Control (0°-30°) =================
            // เพิ่ม/ลดองศา
            if(gamepad2.left_bumper){
                angle += servoStepDeg;
            }
            if(gamepad2.right_bumper){
                angle -= servoStepDeg;
            }

            // จำกัดองศา
            angle = Math.max(angleMin, Math.min(angleMax, angle));

            // แปลงเป็น servo position
            SVR_L0.setPosition(angle / 180.0);        // ฝั่งซ้าย
            SVR_L1.setPosition(1.0 - (angle / 180.0)); // ฝั่งขวา หมุนกลับด้าน
            SVR_sw.setPosition(angle / 180);
            // ================= Telemetry =================
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
