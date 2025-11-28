package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="PedroPathingTeleOpWithIMU", group="Calibration")
public class PedroPathingTeleOpWithIMU extends LinearOpMode {

    DcMotor LF, RF, LR, RR;
    IMU imu;
    ElapsedTime timer = new ElapsedTime();

    // ค่าที่ calibrate แล้ว
    final double forwardTicksToInches = 0.034;
    final double strafeTicksToInches = 0.049;
    final double turnTicksToInches = 0.052;

    @Override
    public void runOpMode() {

        // Motor map
        LF = hardwareMap.get(DcMotor.class, "M_LF");
        RF = hardwareMap.get(DcMotor.class, "M_RF");
        LR = hardwareMap.get(DcMotor.class, "M_LR");
        RR = hardwareMap.get(DcMotor.class, "M_RR");

        LF.setDirection(DcMotor.Direction.FORWARD);
        LR.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addLine("READY");
        telemetry.addLine("A = Forward, B = Strafe, X = Turn");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double forward = 0;
            double strafe = 0;
            double turn = 0;

            // อ่านปุ่ม
            if (gamepad1.a) forward = 1;
            if (gamepad1.b) strafe = 1;
            if (gamepad1.x) turn = 1;

            // แก้เอียงด้วย IMU เวลา Strafe
            double imuCorrection = 0;
            if (strafe != 0) {
                double targetAngle = 0;  // เราต้องการตรง
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double currentAngle = orientation.getYaw(AngleUnit.DEGREES);
                double error = targetAngle - currentAngle;
                double kp = 0.02;  // ปรับตามความแรง
                imuCorrection = error * kp;
            }

            // ตั้ง power ล้อ
            double LFpower = forward + (-strafe) + turn + imuCorrection;
            double LRpower = forward + (strafe) + turn + imuCorrection;
            double RFpower = forward + (strafe) - turn - imuCorrection;
            double RRpower = forward + (-strafe) - turn - imuCorrection;

            // Normalize power > 1
            double max = Math.max(Math.max(Math.abs(LFpower), Math.abs(LRpower)),
                    Math.max(Math.abs(RFpower), Math.abs(RRpower)));
            if (max > 1.0) {
                LFpower /= max;
                LRpower /= max;
                RFpower /= max;
                RRpower /= max;
            }

            // เซ็ต power
            LF.setPower(LFpower);
            LR.setPower(LRpower);
            RF.setPower(RFpower);
            RR.setPower(RRpower);

            // Telemetry
            telemetry.addData("IMU angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("LF", LF.getCurrentPosition());
            telemetry.addData("RF", RF.getCurrentPosition());
            telemetry.addData("LR", LR.getCurrentPosition());
            telemetry.addData("RR", RR.getCurrentPosition());
            telemetry.update();
        }
    }
}
