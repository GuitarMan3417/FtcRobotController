package org.firstinspires.ftc.teamcode.unused;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
@TeleOp(name = "TeleOp: Control (With Apriltags)", group = "TeleOp")
public class AprilTagCodeTeleOP1 extends LinearOpMode {

  DcMotor M_LF, M_RF, M_LR, M_RR;

  // -------------------------------
  // AprilTag Variables
  // -------------------------------
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
    tagProcessor = new AprilTagProcessor.Builder().build();
    visionPortal =
            new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam1"))
                    .addProcessor(tagProcessor)
                    .setCameraResolution(new Size(800, 448))
                    .build();

    telemetry.addLine("Camera opened. Waiting for start...");
    telemetry.update();

    waitForStart();

    while (opModeIsActive()) {

      // =========================================
      //        Auto-Align AprilTag (Press A)
      // =========================================
      if (gamepad1.a) {
        List<AprilTagDetection> tags = tagProcessor.getDetections();
        autoAlignAprilTag(tags, TARGET_TAG_1, TARGET_TAG_2);
        continue;
      }

      // ================================================================
      //              NORMAL DRIVING CONTROL
      // ================================================================
      double forward = -gamepad1.left_stick_y;
      double strafe  =  gamepad1.left_stick_x;
      double rotate  =  gamepad1.right_stick_x;

      double speed = 0.40;

      double powerLF = (forward + strafe + rotate) * speed;
      double powerRF = (forward - strafe - rotate) * speed;
      double powerLR = (forward - strafe + rotate) * speed;
      double powerRR = (forward + strafe - rotate) * speed;

      setDrivePower(powerLF, powerRF, powerLR, powerRR);
    }
  }

  // -------------------------------
  // SMOOTH AUTO-ALIGN FUNCTION
  // -------------------------------
  void autoAlignAprilTag(List<AprilTagDetection> tags, int target1, int target2) {

    AprilTagDetection targetTag = null;
    int frameWidth = 800;

    for (AprilTagDetection tag : tags) {
      if (tag.id == target1 || tag.id == target2) {
        targetTag = tag;
        break;
      }
    }

    // =========================
    // No Tag → Stop
    // =========================
    if (targetTag == null) {
      setDrivePower(0, 0, 0, 0);
      telemetry.addLine("No tag found");
      telemetry.update();
      return;
    }

    double tagX = targetTag.center.x;
    double centerX = frameWidth / 2.0;

    int deadband = 15;  // ถือว่าตรงกลาง

    double error = tagX - centerX;

    // ถ้าอยู่ใน Deadband → หยุด
    if (Math.abs(error) <= deadband) {
      setDrivePower(0, 0, 0, 0);
      telemetry.addLine("Centered ✔");
      telemetry.update();
      return;
    }

    // =============================
    //    SMOOTH ROTATION CONTROL
    // =============================
    double maxPower = 0.30;   // ความเร็วสูงสุด
    double minPower = 0.10;   // ความเร็วขั้นต่ำ
    double K = 0.0018;        // ค่านุ่ม

    double rotatePower = Math.abs(error) * K;

    // จำกัดความเร็ว
    rotatePower = Math.max(minPower, Math.min(maxPower, rotatePower));

    // =========================
    // หมุนตามทิศที่แท็กอยู่
    // =========================
    if (error < 0) {
      setDrivePower(-rotatePower, rotatePower, -rotatePower, rotatePower);
      telemetry.addLine("Tag Left → Rotate Right");
    } else {
      setDrivePower(rotatePower, -rotatePower, rotatePower, -rotatePower);
      telemetry.addLine("Tag Right → Rotate Left");
    }

    telemetry.addData("TagX", tagX);
    telemetry.addData("CenterX", centerX);
    telemetry.addData("Error", error);
    telemetry.addData("RotatePower", rotatePower);
    telemetry.update();
  }

  void setDrivePower(double lf, double rf, double lr, double rr) {
    M_LF.setPower(lf);
    M_RF.setPower(rf);
    M_LR.setPower(lr);
    M_RR.setPower(rr);
  }
}
