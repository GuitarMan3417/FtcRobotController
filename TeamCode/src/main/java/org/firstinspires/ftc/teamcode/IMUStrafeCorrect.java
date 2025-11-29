package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMU_STRAFE_FIX", group = "TeleOp")
public class IMUStrafeCorrect extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor M_LF, M_RF, M_LR, M_RR;
        BNO055IMU imu;
        final double targetAngle = 0;

        // -------------------------
        // Motor Mapping
        // -------------------------
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        // ทิศตามบอทของคุณ (รูปที่ส่งมา)
        M_LF.setDirection(DcMotor.Direction.FORWARD);
        M_RF.setDirection(DcMotor.Direction.FORWARD);
        M_LR.setDirection(DcMotor.Direction.REVERSE);
        M_RR.setDirection(DcMotor.Direction.REVERSE);

        // -------------------------
        // IMU Setup — รองรับวาง Hub แบบที่คุณใช้อยู่
        // USB = ซ้าย, Motor Ports = หลัง
        // ต้องกลับแกนมุมรอบ Z
        // -------------------------
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addLine("Calibrating IMU...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
        }

        telemetry.addLine("READY!");
        telemetry.update();

        waitForStart();

        // -------------------------
        // LOOP
        // -------------------------
        while (opModeIsActive()) {

            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double rotate =  gamepad1.right_stick_x;

            // อ่านมุม IMU
            Orientation angles = imu.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES
            );

            // Control Hub ของคุณวางกลับด้าน → ต้องสลับมุมด้วย (-)
            double currentAngle = -angles.firstAngle;
            double angleError = currentAngle - targetAngle;

            // ค่าแก้เอียง
            double kP = 0.05;
            double correction = angleError * kP;

            // -------------------------
            // สูตร MECANUM ที่ถูกต้อง
            // -------------------------
            double LF = drive + strafe - correction;
            double RF = drive - strafe + correction;
            double LR = drive - strafe - correction;
            double RR = drive + strafe + correction;

            // ใส่หมุนตาม joystick
            LF += rotate;
            RF -= rotate;
            LR += rotate;
            RR -= rotate;

            // Normalize
            double max = Math.max(Math.max(Math.abs(LF), Math.abs(RF)),
                    Math.max(Math.abs(LR), Math.abs(RR)));
            if (max > 1.0) {
                LF /= max;
                RF /= max;
                LR /= max;
                RR /= max;
            }

            // Apply power
            M_LF.setPower(LF);
            M_RF.setPower(RF);
            M_LR.setPower(LR);
            M_RR.setPower(RR);

            // Debug
            telemetry.addData("Angle", currentAngle);
            telemetry.addData("Error", angleError);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }
    }
}
