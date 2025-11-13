package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="FollowBall (OpenCV Auto)", group="Vision")
public class FollowBall extends LinearOpMode {

    // --- มอเตอร์ขับเคลื่อน 2 ข้าง (แทนหุ่น differential drive)
    private DcMotor M_LF, M_RF;

    // --- กล้อง
    OpenCvCamera camera;
    BallDetectPipeline pipeline;

    // --- ค่ากลางของภาพ (ใช้ปรับทิศ)
    static final double CENTER_X = 320; // ถ้าภาพ 640x480

    @Override
    public void runOpMode() throws InterruptedException {
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");

        M_LF.setDirection(DcMotorSimple.Direction.REVERSE); // ให้หมุนทิศเดียวกัน

        // ตั้งค่ากล้อง
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new BallDetectPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
            }
        });

        telemetry.addLine("READY - Press Play");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            BallDetectPipeline.Detection d = pipeline.getLastDetection();

            if (d.found) {
                double error = d.cx - CENTER_X;   // ค่าความคลาดจากกลางภาพ
                double turn = error / 320.0;      // ปรับให้อยู่ในช่วง -1 ถึง 1
                double basePower = 0.25;          // ความเร็วพื้นฐาน

                // ปรับทิศทางเลี้ยวตามตำแหน่งลูกบอล
                double leftPower = basePower - turn * 0.3;
                double rightPower = basePower + turn * 0.3;

                // ถ้าลูกบอลใกล้มาก (รัศมีใหญ่)
                if (d.radius > 80) { // ปรับตามระยะจริง
                    leftPower = 0;
                    rightPower = 0;
                }

                M_LF.setPower(leftPower);
                M_RF.setPower(rightPower);

                telemetry.addData("FOUND", true);
                telemetry.addData("Center", "(%.1f, %.1f)", d.cx, d.cy);
                telemetry.addData("Radius", "%.1f px", d.radius);
                telemetry.addData("Turn Error", "%.1f", error);
            } else {
                // ไม่เจอ → หมุนหา
                M_LF.setPower(-0.2);
                M_RF.setPower(0.2);
                telemetry.addData("FOUND", false);
            }

            telemetry.update();
        }

        // ปิดระบบเมื่อหยุด
        M_LF.setPower(0);
        M_RF.setPower(0);
        camera.stopStreaming();
    }

    // -------------------------
    // Pipeline ตรวจจับลูกบอล
    // -------------------------
    public static class BallDetectPipeline extends OpenCvPipeline {
        private final Object sync = new Object();
        private volatile Detection last = new Detection(false, 0, 0, 0);

        // --- สีเขียว (Green)
        private Scalar lowHSV_green = new Scalar(35, 80, 80);
        private Scalar highHSV_green = new Scalar(85, 255, 255);

        // --- สีม่วง (Purple / Magenta)
        private Scalar lowHSV_purple = new Scalar(130, 80, 80);
        private Scalar highHSV_purple = new Scalar(160, 255, 255);

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat maskGreen = new Mat();
            Mat maskPurple = new Mat();
            Mat maskCombined = new Mat();
            Mat morphed = new Mat();

            Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            // กรองสีเขียว
            Core.inRange(hsv, lowHSV_green, highHSV_green, maskGreen);
            // กรองสีม่วง
            Core.inRange(hsv, lowHSV_purple, highHSV_purple, maskPurple);

            // รวมสองสีเข้าด้วยกัน
            Core.bitwise_or(maskGreen, maskPurple, maskCombined);

            // ทำความสะอาด noise
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
            Imgproc.morphologyEx(maskCombined, morphed, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_CLOSE, kernel);

            // หาขอบและ contour
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            boolean found = false;
            double bestR = 0;
            Point bestC = new Point();

            for (MatOfPoint cnt : contours) {
                double area = Imgproc.contourArea(cnt);
                if (area < 150) continue;
                MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
                Point c = new Point();
                float[] r = new float[1];
                Imgproc.minEnclosingCircle(cnt2f, c, r);

                double circularity = area / (Math.PI * r[0] * r[0]);
                if (circularity > 0.5 && r[0] > bestR) {
                    found = true;
                    bestR = r[0];
                    bestC = c;
                }
            }

            if (found) {
                // วาดวงกลมรอบลูกบอล
                Imgproc.circle(input, bestC, (int) bestR, new Scalar(0, 255, 0), 3);
                Imgproc.putText(input, "BALL", bestC, Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
                synchronized (sync) {
                    last = new Detection(true, bestC.x, bestC.y, bestR);
                }
            } else {
                synchronized (sync) {
                    last = new Detection(false, 0, 0, 0);
                }
            }

            // clear memory
            hsv.release();
            maskGreen.release();
            maskPurple.release();
            maskCombined.release();
            morphed.release();
            hierarchy.release();

            return input;
        }

        public Detection getLastDetection() {
            synchronized (sync) {
                return last;
            }
        }

        public static class Detection {
            public final boolean found;
            public final double cx, cy, radius;

            public Detection(boolean f, double x, double y, double r) {
                found = f;
                cx = x;
                cy = y;
                radius = r;
            }
        }
    }
}