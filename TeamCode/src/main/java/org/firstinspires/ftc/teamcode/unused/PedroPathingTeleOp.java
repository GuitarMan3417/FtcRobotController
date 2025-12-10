package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name = "AutoPedroPathing", group = "Calibration")
public class PedroPathingTeleOp extends LinearOpMode {

    DcMotor LF, RF, LR, RR;

    double distanceInches = 9.5;     // เปลี่ยนระยะได้
    int ticksPerInch = 50;           // ใส่ค่าตามล้อ/encoder ของคุณ (ประมาณไว้ก่อน)

    @Override
    public void runOpMode() throws InterruptedException {

        LF = hardwareMap.get(DcMotor.class, "M_LF");
        RF = hardwareMap.get(DcMotor.class, "M_RF");
        LR = hardwareMap.get(DcMotor.class, "M_LR");
        RR = hardwareMap.get(DcMotor.class, "M_RR");

        // กลับทิศถ้าจำเป็น
        LF.setDirection(DcMotor.Direction.FORWARD);
        LR.setDirection(DcMotor.Direction.FORWARD);

        reset();

        telemetry.addLine("READY Auto Calibration");
        telemetry.addLine("A = Forward 9.5\"");
        telemetry.addLine("B = Strafe 9.5\"");
        telemetry.addLine("X = Turn 360°");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                autoForward(distanceInches);
                showEncoders();
            }

            if (gamepad1.b) {
                autoStrafe(distanceInches);
                showEncoders();
            }

            if (gamepad1.x) {
                autoTurn360();
                showEncoders();
            }

            telemetry.update();
        }
    }

    // =========== FUNCTIONS ============

    void reset() {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ----- 1) Auto Forward -----
    void autoForward(double inches) {
        int ticks = (int)(inches * ticksPerInch);

        setTarget(ticks, ticks, ticks, ticks);
        runToPos(0.4);
    }

    // ----- 2) Auto Strafe -----
    void autoStrafe(double inches) {
        int ticks = (int)(inches * ticksPerInch);

        // mecanum pattern
        setTarget(-ticks, ticks, ticks, -ticks);
        runToPos(0.4);
    }

    // ----- 3) Auto Turn 360° -----
    void autoTurn360() {
        int ticks = (int)(360 * 5);   // 5 ticks/degree (จะ calibrate ทีหลัง)

        setTarget(ticks, -ticks, ticks, -ticks);
        runToPos(0.4);
    }

    void setTarget(int lf, int rf, int lr, int rr) {

        LF.setTargetPosition(lf);
        RF.setTargetPosition(rf);
        LR.setTargetPosition(lr);
        RR.setTargetPosition(rr);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void runToPos(double power) {
        LF.setPower(power);
        RF.setPower(power);
        LR.setPower(power);
        RR.setPower(power);

        while (opModeIsActive() &&
                (LF.isBusy() || RF.isBusy() || LR.isBusy() || RR.isBusy())) {
            showEncoders();
        }

        stopMotors();
        reset();
    }

    void stopMotors() {
        LF.setPower(0);
        RF.setPower(0);
        LR.setPower(0);
        RR.setPower(0);
    }

    void showEncoders() {
        telemetry.addLine("------ Encoder Values ------");
        telemetry.addData("LF", LF.getCurrentPosition());
        telemetry.addData("RF", RF.getCurrentPosition());
        telemetry.addData("LR", LR.getCurrentPosition());
        telemetry.addData("RR", RR.getCurrentPosition());
        telemetry.addLine("-----------------------------");
    }
}
