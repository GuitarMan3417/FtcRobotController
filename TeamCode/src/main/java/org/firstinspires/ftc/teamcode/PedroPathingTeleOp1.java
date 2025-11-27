package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PedroPathingTeleOp1", group = "Calibration")
public class PedroPathingTeleOp1 extends LinearOpMode {

    DcMotor LF, RF, LR, RR;

    @Override
    public void runOpMode() throws InterruptedException {

        LF = hardwareMap.get(DcMotor.class, "M_LF");
        RF = hardwareMap.get(DcMotor.class, "M_RF");
        LR = hardwareMap.get(DcMotor.class, "M_LR");
        RR = hardwareMap.get(DcMotor.class, "M_RR");

        // ถ้าล้อคุณต้องกลับทิศ ให้แก้ตรงนี้
        LF.setDirection(DcMotor.Direction.FORWARD);
        LR.setDirection(DcMotor.Direction.FORWARD);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("READY to Calibrate");
        telemetry.addLine("A = Forward");
        telemetry.addLine("B = Strafe");
        telemetry.addLine("X = Turn (Spin)");
        telemetry.addLine("Press START to continue...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========= FORWARD (A) =========
            if (gamepad1.a) {
                driveForward(0.3);
            }

            // ========= STRAFE (B) =========
            else if (gamepad1.b) {
                driveStrafe(0.3);
            }

            // ========= TURN (X) =========
            else if (gamepad1.x) {
                driveTurn(0.3);
            }

            // ========= STOP =========
            else {
                stopMotors();
            }

            // แสดงค่า Encoder ตลอด
            telemetry.addData("LF", LF.getCurrentPosition());
            telemetry.addData("RF", RF.getCurrentPosition());
            telemetry.addData("LR", LR.getCurrentPosition());
            telemetry.addData("RR", RR.getCurrentPosition());
            telemetry.update();
        }
    }

    // ========== METHODS ==========

    // ขับตรง
    void driveForward(double p) {
        LF.setPower(p);
        LR.setPower(p);
        RF.setPower(p);
        RR.setPower(p);
    }

    // ขับ Strafe
    void driveStrafe(double p) {
        LF.setPower(-p);
        LR.setPower(p);
        RF.setPower(p);
        RR.setPower(-p);
    }

    // หมุนตัว
    void driveTurn(double p) {
        LF.setPower(p);
        LR.setPower(p);
        RF.setPower(-p);
        RR.setPower(-p);
    }

    // หยุดล้อหมด
    void stopMotors() {
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
    }
}
