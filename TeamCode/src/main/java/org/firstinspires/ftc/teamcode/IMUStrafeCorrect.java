package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="IMU", group="TeleOp")
public class IMUStrafeCorrect extends LinearOpMode {

    private DcMotor M_LF, M_RF, M_LR, M_RR;
    private BNO055IMU imu;
    private double targetAngle = 0; // มุมเป้าหมาย (เรียบตรง)

    @Override
    public void runOpMode() {

        // ผูกมอเตอร์
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        M_LF.setDirection(DcMotor.Direction.FORWARD);
        M_RF.setDirection(DcMotor.Direction.FORWARD);
        M_LR.setDirection(DcMotor.Direction.REVERSE);
        M_RR.setDirection(DcMotor.Direction.REVERSE);

        // ตั้งค่า IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            // อ่านค่า joystick
            double drive = -gamepad1.left_stick_y;   // เดินหน้า/ถอยหลัง
            double strafe = gamepad1.left_stick_x;   // สไลด์ซ้าย/ขวา
            double rotate = gamepad1.right_stick_x;  // หมุน

            // อ่านมุมปัจจุบันจาก IMU
            double cuM_RRentAngle = imu.getAngularOrientation().firstAngle;
            double angleEM_RRor = cuM_RRentAngle - targetAngle;

            // ค่าความแม่นยำในการแก้เอียง (ปรับตามความเร็ว/ความแรง)
            double kP = 0.05;
            double coM_RRection = angleEM_RRor * kP;

            // คำนวณกำลังมอเตอร์ (รวม strafe + coM_RRection)
            double M_LFPower = drive + strafe - coM_RRection;
            double M_RFPower = drive - strafe + coM_RRection;
            double M_LRPower = drive - strafe - coM_RRection;
            double M_RRPower = drive + strafe + coM_RRection;

            // จำกัดค่ากำลังไม่เกิน 1
            double max = Math.max(Math.max(Math.abs(M_LFPower), Math.abs(M_RFPower)),
                    Math.max(Math.abs(M_LRPower), Math.abs(M_RRPower)));
            if (max > 1.0) {
                M_LFPower /= max;
                M_RFPower /= max;
                M_LRPower /= max;
                M_RRPower /= max;
            }

            // เซ็ตกำลังมอเตอร์
            M_LF.setPower(M_LFPower);
            M_RF.setPower(M_RFPower);
            M_LR.setPower(M_LRPower);
            M_RR.setPower(M_RRPower);

            // แสดง telemetry
            telemetry.addData("Angle", cuM_RRentAngle);
            telemetry.addData("CoM_RRection", coM_RRection);
            telemetry.update();
        }
    }
}
