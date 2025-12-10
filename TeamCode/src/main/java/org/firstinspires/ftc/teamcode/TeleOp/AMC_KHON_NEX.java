package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Disabled
@TeleOp(name="TeleOp: Control + AprilTags", group="TeleOp")
public class AMC_KHON_NEX extends LinearOpMode {

    DcMotor M_AIN, M_S0, M_S1, M_bl, M_LF, M_RF, M_LR, M_RR;
    Servo SVR_L0, SVR_L1, SVR_sw;

    double speedMultiplier = 0.40;
    private double shootingPower1 = 0.46;
    private double shootingPower2 = 0;

    // ---------------------------
    // AprilTag Variables
    // ---------------------------
    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;

    int TARGET_TAG_1 = 20;
    int TARGET_TAG_2 = 24;

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
        M_S0  = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1  = hardwareMap.get(DcMotor.class, "M_S1");
        M_bl  = hardwareMap.get(DcMotor.class, "M_bl");

        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");
        SVR_sw = hardwareMap.get(Servo.class, "SVR_sw");

        // Motor Directions
        M_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        M_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        M_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_RR.setDirection(DcMotorSimple.Direction.REVERSE);

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        SVR_L0.setPosition(0.46);
        SVR_L1.setPosition(0);
        SVR_sw.setPosition(0.1);

        // =============================
        //  Setup AprilTag Camera
        // =============================
        tagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal =
                new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "webcam1"))
                        .addProcessor(tagProcessor)
                        .setCameraResolution(new Size(800, 448))
                        .build();

        telemetry.addLine("Camera Ready. Waiting for Start...");
        telemetry.update();

        waitForStart();

        // =============================
        //  MAIN LOOP
        // =============================
        while (opModeIsActive()) {

            // AUTO-ALIGN (press A)
            if (gamepad1.a) {
                List<AprilTagDetection> tags = tagProcessor.getDetections();
                autoAlignAprilTag(tags, TARGET_TAG_1, TARGET_TAG_2);
                continue;
            }

            // =======================================================
            //        NORMAL SERVO CONTROL (Shooter angle)
            // =======================================================
            if (gamepad2.right_bumper) {
                shootingPower1 -= 0.001;
                shootingPower2 += 0.001;
            }
            if (gamepad2.left_bumper) {
                shootingPower1 += 0.001;
                shootingPower2 -= 0.001;
            }

            shootingPower1 = Math.max(0.20, Math.min(0.46, shootingPower1));
            shootingPower2 = Math.max(0.00, Math.min(0.20, shootingPower2));

            SVR_L0.setPosition(shootingPower1);
            SVR_L1.setPosition(shootingPower2);

            SVR_sw.setPosition(gamepad2.y ? 0.5 : 0.1);

            // =======================================================
            //                 Intake Motor
            // =======================================================
            double intakePower = 0.18; // idle

            if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1)
                intakePower = -0.65;

            else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1)
                intakePower = 0.55;

            M_AIN.setPower(intakePower);

            // Spin Up Motor
            if (gamepad2.a) M_S0.setPower(1);
            else if (gamepad2.x) M_S0.setPower(-1);
            else M_S0.setPower(0);

            // Shooting Logic (blocking + shooter)
            handleShooting();

            // =======================================================
            //           NORMAL MECANUM DRIVING
            // =======================================================
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double rotate  =  gamepad1.right_stick_x;

            speedMultiplier = (Math.abs(forward) < 0.001) ? 0.35 : 0.50;

            double lf = (forward + strafe + rotate) * speedMultiplier;
            double rf = (forward - strafe - rotate) * speedMultiplier;
            double lr = (forward - strafe + rotate) * speedMultiplier;
            double rr = (forward + strafe - rotate) * speedMultiplier;

            double max = Math.max(1.0, Math.max(Math.abs(lf),
                    Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr)))));

            setDrivePower(lf/max, rf/max, lr/max, rr/max);

            telemetry.update();
        }
    }

    // =============================
    //  SHOOTING SYSTEM
    // =============================
    void handleShooting() {
        if (gamepad2.b || gamepad2.left_stick_y < -0.2) {
            M_S1.setPower(-0.9);
            M_S0.setPower(1);

            sleep(600);
            M_bl.setPower(-1);
        } else {
            M_bl.setPower(0);
            M_S1.setPower(-0.35);
        }
    }

    // =============================
    //  APRILTAGS AUTO ALIGN
    // =============================
    void autoAlignAprilTag(List<AprilTagDetection> tags, int target1, int target2) {

        AprilTagDetection targetTag = null;
        int frameWidth = 800;

        for (AprilTagDetection tag : tags) {
            if (tag.id == target1 || tag.id == target2) {
                targetTag = tag;
                break;
            }
        }

        if (targetTag == null) {
            setDrivePower(0, 0, 0, 0);
            telemetry.addLine("No AprilTag Found");
            return;
        }

        double tagX = targetTag.center.x;
        double centerX = frameWidth / 2.0;

        double error = tagX - centerX;
        int deadband = 15;

        if (Math.abs(error) <= deadband) {
            setDrivePower(0, 0, 0, 0);
            telemetry.addLine("Centered ✔");
            return;
        }

        double maxPower = 0.30;
        double minPower = 0.10;
        double K = 0.0018;

        double rotatePower = Math.abs(error) * K;
        rotatePower = Math.max(minPower, Math.min(maxPower, rotatePower));

        if (error < 0)
            setDrivePower(-rotatePower, rotatePower, -rotatePower, rotatePower);
        else
            setDrivePower(rotatePower, -rotatePower, rotatePower, -rotatePower);
        telemetry.addLine("--- AMC KHON NEX ---");
        telemetry.addData("M_AIN", M_AIN.getPower());
        telemetry.addData("M_S0", M_S0.getPower());
        telemetry.addData("M_bl", M_bl.getPower());
        telemetry.addData("M_S1", M_S1.getPower());
        telemetry.addData("SVR_L0", SVR_L0.getPosition());
        telemetry.addData("SVR_L1", SVR_L1.getPosition());
        telemetry.addData("SVR_sw", SVR_sw.getPosition());
        telemetry.addData("Power LF", M_LF.getPower());
        telemetry.addData("Power RF", M_RF.getPower());
        telemetry.addData("Power LR", M_LR.getPower());
        telemetry.addData("Power RR", M_RR.getPower());
        telemetry.addData("TagX", tagX);
        telemetry.addData("Error", error);
        telemetry.addData("Rotate Power", rotatePower);
        telemetry.update();
    }

    // =============================
    //  DRIVE POWER FUNCTION
    // =============================
    void setDrivePower(double lf, double rf, double lr, double rr) {
        M_LF.setPower(lf);
        M_RF.setPower(rf);
        M_LR.setPower(lr);
        M_RR.setPower(rr);
    }
}
