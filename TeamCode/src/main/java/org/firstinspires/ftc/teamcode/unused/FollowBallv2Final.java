package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled

@TeleOp(name="FollowBall Mecanum FINAL", group="Vision")
public class FollowBallv2Final extends LinearOpMode {

    private DcMotor M_LF, M_RF, M_LR, M_RR;

    OpenCvCamera camera;
    BallDetectPipeline pipeline;

    static final double CENTER_X = 320;

    @Override
    public void runOpMode() throws InterruptedException {

        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        M_LF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_LR.setDirection(DcMotorSimple.Direction.REVERSE);

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
            }
        });

        telemetry.addLine("READY - Press Play");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            BallDetectPipeline.Detection d = pipeline.getLastDetection();
            double[] hsv = pipeline.getLastHSV();

            double forward = 0;
            double strafe = 0;
            double rotate = 0;

            if (d.found) {
                double error = d.cx - CENTER_X;
                strafe = error / 320.0;
                forward = 0.3;
                rotate = 0;

                if (d.radius > 80) {
                    forward = strafe = rotate = 0;
                }
            } else {
                forward = 0;
                strafe = 0;
                rotate = 0.3;
            }

            double lf = forward + strafe + rotate;
            double rf = forward - strafe - rotate;
            double lr = forward - strafe + rotate;
            double rr = forward + strafe - rotate;

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

            // ─────────────────────────────
            //      TELEMETRY TO DRIVER HUB
            // ─────────────────────────────
            telemetry.addLine("=== CAMERA STATUS ===");
            telemetry.addData("Streaming", camera.getFps() > 0);
            telemetry.addData("Resolution", "640x480");

            telemetry.addLine();
            telemetry.addLine("=== BALL DETECTION ===");
            telemetry.addData("Found", d.found);
            if (d.found) {
                telemetry.addData("Color", d.colorName);
                telemetry.addData("Center X", "%.1f", d.cx);
                telemetry.addData("Center Y", "%.1f", d.cy);
                telemetry.addData("Radius", "%.1f px", d.radius);
                telemetry.addData("Error", "%.1f", d.cx - CENTER_X);
            }

            telemetry.addLine();
            telemetry.addLine("=== HSV CENTER ===");
            if (hsv != null)
                telemetry.addData("H,S,V", "%.0f  %.0f  %.0f", hsv[0], hsv[1], hsv[2]);

            telemetry.update();
        }

        camera.stopStreaming();
    }


    //----------------------------------------------------
    //           PIPELINE (GREEN + PURPLE ONLY)
    //----------------------------------------------------
    public static class BallDetectPipeline extends OpenCvPipeline {

        private final Object sync = new Object();
        private volatile Detection last = new Detection(false, 0, 0, 0, "");
        private volatile double[] lastHSV = null;

        // HSV ช่วงใหม่ (กันสีเพี้ยนจากไฟสนาม)
        private Scalar lowHSV_green = new Scalar(30, 60, 60);
        private Scalar highHSV_green = new Scalar(95, 255, 255);

        private Scalar lowHSV_purple = new Scalar(110, 70, 50);
        private Scalar highHSV_purple = new Scalar(179, 255, 255);

        @Override
        public Mat processFrame(Mat input) {

            // ─────────────────────────────────────
            //   1) ลด Noise สีเพี้ยนด้วย Blur 3x3
            // ─────────────────────────────────────
            Imgproc.GaussianBlur(input, input, new Size(3, 3), 0);

            // ─────────────────────────────────────
            //   2) BGR → HSV
            // ─────────────────────────────────────
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            // ─────────────────────────────────────
            //   3) Normalize ช่วยกันไฟม่วง/ไฟสนาม
            // ─────────────────────────────────────
            Core.normalize(hsv, hsv, 0, 255, Core.NORM_MINMAX);

            // ─────────────────────────────────────
            //   4) ทำ Mask จากสีเขียว/ม่วง
            // ─────────────────────────────────────
            Mat maskGreen = new Mat();
            Mat maskPurple = new Mat();

            Core.inRange(hsv, lowHSV_green, highHSV_green, maskGreen);
            Core.inRange(hsv, lowHSV_purple, highHSV_purple, maskPurple);

            // ─────────────────────────────────────
            //   5) Morphology ลด noise → วงกลมชัดขึ้น
            // ─────────────────────────────────────
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
            Imgproc.morphologyEx(maskGreen, maskGreen, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskGreen, maskGreen, Imgproc.MORPH_CLOSE, kernel);

            Imgproc.morphologyEx(maskPurple, maskPurple, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskPurple, maskPurple, Imgproc.MORPH_CLOSE, kernel);

            // ─────────────────────────────────────
            //   6) หา Contours แล้วฟิตวงกลม
            // ─────────────────────────────────────
            boolean found = false;
            double bestR = 0;
            Point bestC = new Point();
            Scalar bestColor = new Scalar(255, 255, 255);
            String bestName = "";

            List<Mat> masks = List.of(maskGreen, maskPurple);
            List<Scalar> colors = List.of(new Scalar(0, 255, 0), new Scalar(255, 0, 255));
            List<String> names = List.of("GREEN", "PURPLE");

            for (int i = 0; i < masks.size(); i++) {
                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();

                Imgproc.findContours(masks.get(i), contours, hierarchy,
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                for (MatOfPoint cnt : contours) {
                    double area = Imgproc.contourArea(cnt);
                    if (area < 150) continue;

                    MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
                    Point c = new Point();
                    float[] r = new float[1];

                    Imgproc.minEnclosingCircle(cnt2f, c, r);

                    double circularity = area / (Math.PI * r[0] * r[0]);
                    if (circularity > 0.50 && r[0] > bestR) {
                        found = true;
                        bestR = r[0];
                        bestC = c;
                        bestColor = colors.get(i);
                        bestName = names.get(i);
                    }
                }
            }

            // ─────────────────────────────────────
            //   7) วาดผล + ส่งข้อมูลกลับไปให้ Main code
            // ─────────────────────────────────────
            if (found) {
                Imgproc.circle(input, bestC, (int) bestR, bestColor, 3);
                Imgproc.putText(input, bestName,
                        new Point(bestC.x - 30, bestC.y - bestR - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, bestColor, 2);

                synchronized (sync) {
                    last = new Detection(true, bestC.x, bestC.y, bestR, bestName);
                }
            } else {
                synchronized (sync) {
                    last = new Detection(false, 0, 0, 0, "");
                }
            }

            // ─────────────────────────────────────
            //   8) HSV ตรงกลางภาพ → Debug สีเพี้ยน
            // ─────────────────────────────────────
            double[] hsvCenter = hsv.get(hsv.rows() / 2, hsv.cols() / 2);
            if (hsvCenter != null) {
                lastHSV = hsvCenter;

                Imgproc.putText(input,
                        String.format("HSV %.0f %.0f %.0f", hsvCenter[0], hsvCenter[1], hsvCenter[2]),
                        new Point(10, 30),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        new Scalar(0, 255, 255),
                        2);
            }

            // Crosshair
            Imgproc.drawMarker(input, new Point(320, 240),
                    new Scalar(255, 255, 0), Imgproc.MARKER_CROSS, 40, 2);

            hsv.release();
            maskGreen.release();
            maskPurple.release();

            return input;
        }

        public double[] getLastHSV() {
            return lastHSV;
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