package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@SuppressWarnings("unused")
@TeleOp(name = "TeleOp: Control (With Apriltags)", group = "TeleOp")
public class AprilTagCodeTeleOP1 extends LinearOpMode {

  DcMotor M_AIN, M_S0, M_S1, M_bl, M_LF, M_RF, M_LR, M_RR;
  Servo SVR_L0, SVR_L1;

  // -------------------------------
  // AprilTag Variables
  // -------------------------------
  VisionPortal visionPortal;
  AprilTagProcessor tagProcessor;

  int TARGET_TAG_1 = 20;
  int TARGET_TAG_2 = 23;

  // PID constants
  //  double kP = 0.003;
  //  double kI = 0.00001;
  //  double kD = 0.0005;
  //  double integral = 0;
  //  double lastError = 0;

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
        continue; // skip normal drive control while auto-aligning
      }

      // ================================================================
      //              NORMAL DRIVING CONTROL (เหมือนเดิม)
      // ================================================================
      double forward = -gamepad1.left_stick_y;
      double strafe = gamepad1.left_stick_x;
      double rotate = gamepad1.right_stick_x;

      double speed = 0.40;

      double powerLF = (forward + strafe + rotate) * speed;
      double powerRF = (forward - strafe - rotate) * speed;
      double powerLR = (forward - strafe + rotate) * speed;
      double powerRR = (forward + strafe - rotate) * speed;

      setDrivePower(powerLF, powerRF, powerLR, powerRR);
    }
  }

  // -------------------------------
  // PID Auto-Align Function
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
    // ไม่เจอแท็ก → หยุด
    // =========================
    if (targetTag == null) {
      setDrivePower(0, 0, 0, 0);
      telemetry.addLine("No tag found");
      telemetry.update();
      return;
    }

    double tagX = targetTag.center.x;
    double centerX = frameWidth / 2.0;

    // ระยะที่ถือว่า "ตรงกลางแล้ว"
    int deadband = 15;

    double error = tagX - centerX;

    // ตรงกลางแล้ว → หยุดหมุนเลย
    if (Math.abs(error) <= deadband) {
      setDrivePower(0, 0, 0, 0);
      telemetry.addLine("Centered ✔");
      telemetry.update();
      return;
    }

    // =========================
    //  ทิศทางหมุนแบบชัดเจน
    // =========================
    double rotatePower = 0.25;  // ความเร็วหมุนคงที่

    if (error < 0) {
      // แท็กอยู่ซ้ายของภาพ → หมุนขวา
      setDrivePower(rotatePower, -rotatePower, rotatePower, -rotatePower);
      telemetry.addLine("Tag Left → Rotate Right");
    }
    else {
      // แท็กอยู่ขวาของภาพ → หมุนซ้าย
      setDrivePower(-rotatePower, rotatePower, -rotatePower, rotatePower);
      telemetry.addLine("Tag Right → Rotate Left");
    }

    telemetry.addData("TagX", tagX);
    telemetry.addData("CenterX", centerX);
    telemetry.addData("Error", error);
    telemetry.update();
  }

  void setDrivePower(double lf, double rf, double lr, double rr) {
    M_LF.setPower(lf);
    M_RF.setPower(rf);
    M_LR.setPower(lr);
    M_RR.setPower(rr);
  }
}
