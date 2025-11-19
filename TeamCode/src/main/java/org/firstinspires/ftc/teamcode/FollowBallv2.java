package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.openftc.easyopencv.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.Scalar;


import java.util.ArrayList;
import java.util.List;
@Disabled

@TeleOp(name="FollowBall Mecanum (OpenCV)", group="Vision")
public class FollowBallv2 extends LinearOpMode {

    // --- มอเตอร์ Mecanum 4 ล้อ
    private DcMotor M_LF, M_RF, M_LR, M_RR;

    // --- กล้อง
    OpenCvCamera camera;
    BallDetectPipeline pipeline;

    // --- ค่ากลางของภาพ (ใช้ปรับทิศ)
    static final double CENTER_X = 320; // ถ้าภาพ 640x480

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Map มอเตอร์
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        M_LF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_LR.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- ตั้งค่ากล้อง
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

            double forward = 0;
            double strafe = 0;
            double rotate = 0;

            if (d.found) {
                double error = d.cx - CENTER_X;   // ค่าความคลาดจากกลางภาพ
                strafe = error / 320.0;           // ปรับให้เป็น -1..1
                forward = 0.3;                     // ความเร็วเดินหน้า
                rotate = 0;

                // หยุดถ้าลูกบอลใกล้
                if (d.radius > 80) {
                    forward = strafe = rotate = 0;
                }

                telemetry.addData("FOUND", true);
                telemetry.addData("Color", d.colorName);
                telemetry.addData("Center", "(%.1f, %.1f)", d.cx, d.cy);
                telemetry.addData("Radius", "%.1f px", d.radius);
                telemetry.addData("Strafe Error", "%.1f", error);
            } else {
                // ไม่เจอ → หมุนหา
                forward = 0;
                strafe = 0;
                rotate = 0.3; // หมุนตัวเองช้า
                telemetry.addData("FOUND", false);
            }

            // --- Mecanum drive power
            double lf = forward + strafe + rotate;
            double rf = forward - strafe - rotate;
            double lr = forward - strafe + rotate;
            double rr = forward + strafe - rotate;

            // --- Limit power -1..1
            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lr), Math.abs(rr)));
            if (max > 1.0) {
                lf /= max;
                rf /= max;
                lr /= max;
                rr /= max;
            }

            M_LF.setPower(lf);
            M_RF.setPower(rf);
            M_LR.setPower(lr);
            M_RR.setPower(rr);

            telemetry.update();
        }

        // ปิดระบบ
        M_LF.setPower(0);
        M_RF.setPower(0);
        M_LR.setPower(0);
        M_RR.setPower(0);
        camera.stopStreaming();
    }

    // -------------------------
    // Pipeline ตรวจจับลูกบอล
    // -------------------------
    public static class BallDetectPipeline extends OpenCvPipeline {
        private final Object sync = new Object();
        private volatile Detection last = new Detection(false, 0, 0, 0, "");

        // --- สีเขียว (Green)
        private Scalar lowHSV_green = new Scalar(35, 80, 80);
        private Scalar highHSV_green = new Scalar(85, 255, 255);

        // --- สีม่วง (Purple / Magenta)
        private Scalar lowHSV_purple = new Scalar(117, 120, 75);
        private Scalar highHSV_purple = new Scalar(179, 255, 255);



        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat maskGreen = new Mat();
            Mat maskPurple = new Mat();
            Mat maskBlue = new Mat();

            Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            // กรองสี
            Core.inRange(hsv, lowHSV_green, highHSV_green, maskGreen);
            Core.inRange(hsv, lowHSV_purple, highHSV_purple, maskPurple);

            // จัดเก็บ mask / สี / ชื่อ
            List<Mat> masks = new ArrayList<>();
            List<Scalar> colors = new ArrayList<>();
            List<String> names = new ArrayList<>();

            masks.add(maskGreen);
            colors.add(new Scalar(0, 255, 0));
            names.add("GREEN");
            masks.add(maskPurple);
            colors.add(new Scalar(255, 0, 255));
            names.add("PURPLE");
            masks.add(maskBlue);


            boolean found = false;
            double bestR = 0;
            Point bestC = new Point();
            Scalar bestColor = new Scalar(255, 255, 255);
            String bestName = "";

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

            for (int i = 0; i < masks.size(); i++) {
                Mat mask = new Mat();
                Imgproc.morphologyEx(masks.get(i), mask, Imgproc.MORPH_OPEN, kernel);
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
                        bestColor = colors.get(i);
                        bestName = names.get(i);
                    }
                }

                mask.release();
                hierarchy.release();
            }

            if (found) {
                Imgproc.circle(input, bestC, (int) bestR, bestColor, 3);
                Imgproc.putText(input, bestName, new Point(bestC.x - 30, bestC.y - bestR - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, bestColor, 2);
                synchronized (sync) {
                    last = new Detection(true, bestC.x, bestC.y, bestR, bestName);
                }
            } else {
                synchronized (sync) {
                    last = new Detection(false, 0, 0, 0, "");
                }
            }

            // ---- DEBUG HSV ----
            double[] hsvCenter = hsv.get(hsv.rows() / 2, hsv.cols() / 2);
            if (hsvCenter != null) {
                Imgproc.putText(
                        input,
                        String.format("HSV: %.0f %.0f %.0f", hsvCenter[0], hsvCenter[1], hsvCenter[2]),
                        new Point(10, 30),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        new Scalar(0, 255, 255),
                        2
                );
            }

            hsv.release();
            maskGreen.release();
            maskPurple.release();
            maskBlue.release();

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
            public final String colorName;

            public Detection(boolean f, double x, double y, double r, String name) {
                found = f;
                cx = x;
                cy = y;
                radius = r;
                colorName = name;
            }
        }
    }
}
