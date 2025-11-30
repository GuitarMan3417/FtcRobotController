package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// ========== Camera + AprilTag ==========
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="TeleOp: Control", group = "TeleOp")
public class AprilTagCodeTeleOP1 extends LinearOpMode {

    DcMotor M_AIN, M_S0, M_S1, M_bl, M_LF, M_RF, M_LR, M_RR;
    Servo SVR_L0, SVR_L1;

    // -------------------------------
    // AprilTag Variables
    // -------------------------------
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    int TARGET_TAG_1 = 20;
    int TARGET_TAG_2 = 23;

    @Override
    public void runOpMode() throws InterruptedException {

        // =============================
        //  Hardware Mapping
        // =============================
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

        // --------------------------
        // Mecanum Motor Directions
        // --------------------------
        M_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // =============================
        //       Setup AprilTag
        // =============================
        initAprilTag();

        telemetry.addLine("Camera opened. Waiting for start...");
        telemetry.update();

        // =============================
        //          START
        // =============================
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // =========================================
                //        AprilTag Auto Align (Press A)
                // =========================================
                List<AprilTagDetection> tags = tagProcessor.getDetections();

                boolean foundTag = false;
                double tagX = 0;

                for (AprilTagDetection tag : tags) {
                    if (tag.id == TARGET_TAG_1 || tag.id == TARGET_TAG_2) {
                        foundTag = true;
                        tagX = tag.center.x;   // center X pixel of detected tag
                        break;
                    }
                }

                if (gamepad1.a && foundTag) {
                    int centerX = 400;    // image center (for 800px width)
                    double error = tagX - centerX;

                    double rotatePower = 0;

                    if (Math.abs(error) > 20) {  // tolerance
                        rotatePower = error > 0 ? -0.30 : 0.30;
                    }

                    M_LF.setPower(rotatePower);
                    M_RF.setPower(-rotatePower);
                    M_LR.setPower(rotatePower);
                    M_RR.setPower(-rotatePower);

                    telemetry.addLine("Auto Aligning to AprilTag...");
                    telemetry.addData("Tag X", tagX);
                    telemetry.addData("Error", error);
                    telemetry.update();

                    continue; // skip normal drive control while aligning
                }

                // ================================================================
                //              NORMAL DRIVING CONTROL (เหมือนเดิม)
                // ================================================================
                double forward = -gamepad1.left_stick_y;
                double strafe  = gamepad1.left_stick_x;
                double rotate  = gamepad1.right_stick_x;

                double speed = 0.40;

                double powerLF = (forward + strafe + rotate) * speed;
                double powerRF = (forward - strafe - rotate) * speed;
                double powerLR = (forward - strafe + rotate) * speed;
                double powerRR = (forward + strafe - rotate) * speed;

                M_LF.setPower(powerLF);
                M_RF.setPower(powerRF);
                M_LR.setPower(powerLR);
                M_RR.setPower(powerRR);

                // ---------------- telemetry ----------------
                telemetry.addLine("TeleOp Running");
                telemetry.addData("Forward", forward);
                telemetry.addData("Strafe", strafe);
                telemetry.addData("Rotate", rotate);
                telemetry.addLine("---- Tag Info ----");
                telemetry.addData("FoundTag", foundTag);
                telemetry.addData("TagX", tagX);
                telemetry.update();
            }
        }
        visionPortal.close();
    }

    private void initAprilTag() {
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam1"), tagProcessor);
    }
}