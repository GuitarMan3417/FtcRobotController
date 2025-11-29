package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="IMU_STRAFE_FIX", group="TeleOp")
public class IMUStrafeCorrect extends LinearOpMode {

    private DcMotor M_LF, M_RF, M_LR, M_RR;
    private IMU imu;

    private double targetAngle = 0;  // ต้องการให้ตรง

    @Override
    public void runOpMode() {

        // ------------------------------
        //   Motor Setup
        // ------------------------------
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        // ค่าตามรูปการวางล้อของคุณ — ใช้ตามเดิมได้เลย
        M_LF.setDirection(DcMotor.Direction.FORWARD);
        M_RF.setDirection(DcMotor.Direction.FORWARD);
        M_LR.setDirection(DcMotor.Direction.REVERSE);
        M_RR.setDirection(DcMotor.Direction.REVERSE);

        // ------------------------------
        //   IMU Setup
        // ------------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        // The orientation of the hub is with the USB ports facing left.
        // We also assume the logo is facing up.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        telemetry.addLine("IMU Initializing...");
        telemetry.update();

        // SDK < 8.2: while (!isStopRequested() && !imu.isGyroCalibrated())
        // SDK >= 8.2: No calibration needed for IMU class.

        telemetry.addLine("READY!");
        telemetry.update();

        waitForStart();

        // ------------------------------
        //   CONTROL LOOP
        // ------------------------------
        while (opModeIsActive()) {

            double drive  = -gamepad1.left_stick_y;  // เดินหน้า
            double strafe =  gamepad1.left_stick_x;  // สไลด์
            double rotate =  gamepad1.right_stick_x; // หมุน

            // อ่านค่า IMU (Yaw)
            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            double currentAngle = robotOrientation.getYaw(AngleUnit.DEGREES);

            double angleError = currentAngle - targetAngle;

            // ค่าแก้เอียง
            double kP = 0.05;
            double correction = angleError * kP;

            // รวมแก้เอียงกับ strafe ขับทั่วไป
            double LF = drive + strafe - correction + rotate;
            double RF = drive - strafe + correction - rotate;
            double LR = drive - strafe - correction + rotate;
            double RR = drive + strafe + correction - rotate;

            // Normalize
            double max = Math.max(Math.max(Math.abs(LF), Math.abs(RF)),
                    Math.max(Math.abs(LR), Math.abs(RR)));
            if (max > 1.0) {
                LF /= max;
                RF /= max;
                LR /= max;
                RR /= max;
            }

            // Set power
            M_LF.setPower(LF);
            M_RF.setPower(RF);
            M_LR.setPower(LR);
            M_RR.setPower(RR);

            // Display data
            telemetry.addData("Angle", currentAngle);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Error", angleError);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }
    }
}
