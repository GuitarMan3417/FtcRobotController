package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="FollowBall + AprilTag FINAL", group="Vision")
public class FollowBallAprilTagFinal extends LinearOpMode {

    private DcMotor M_LF, M_RF, M_LR, M_RR;

    private VisionPortal visionPortal;
    private BallDetectionProcessor ballDetectionProcessor;
    private AprilTagProcessor aprilTagProcessor;

    static final double CENTER_X = 320;

    @Override
    public void runOpMode() throws InterruptedException {

        // ───────────────────────────────────────────
        //              MOTOR SETUP
        // ───────────────────────────────────────────
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");

        M_LF.setDirection(DcMotorSimple.Direction.REVERSE);
        M_LR.setDirection(DcMotorSimple.Direction.REVERSE);

        // ───────────────────────────────────────────
        //              CAMERA SETUP
        // ───────────────────────────────────────────
        ballDetectionProcessor = new BallDetectionProcessor();
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTagProcessor, ballDetectionProcessor)
                .build();


        telemetry.addLine("READY - Press PLAY");
        telemetry.update();
        waitForStart();


        // ───────────────────────────────────────────
        //                 MAIN LOOP
        // ───────────────────────────────────────────
        while (opModeIsActive()) {

            BallDetectionProcessor.BallDetection ball = ballDetectionProcessor.getLastBall();
            
            List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getDetections();
            AprilTagDetection tag = null;
            if (!aprilTagDetections.isEmpty()){
                tag = aprilTagDetections.get(0);
            }

            double[] hsv = ballDetectionProcessor.getLastHSV();

            double forward = 0, strafe = 0, rotate = 0;

            // ───────────────────────────────────────────
            //           FOLLOW BALL LOGIC
            // ───────────────────────────────────────────
            if (ball.found) {
                double error = ball.cx - CENTER_X;
                strafe = error / 320.0;
                forward = 0.3;
                rotate = 0;

                if (ball.r > 80) {
                    forward = 0;
                    strafe = 0;
                }
            } else {
                rotate = 0.3;   // หมุนหาลูก
            }

            // ───────────────────────────────────────────
            //           MECANUM DRIVE MIXING
            // ───────────────────────────────────────────
            double lf = forward + strafe + rotate;
            double rf = forward - strafe - rotate;
            double lr = forward - strafe + rotate;
            double rr = forward + strafe - rotate;

            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lr), Math.abs(rr)));
            if (max > 1) {
                lf /= max; rf /= max; lr /= max; rr /= max;
            }

            M_LF.setPower(lf);
            M_RF.setPower(rf);
            M_LR.setPower(lr);
            M_RR.setPower(rr);


            // ───────────────────────────────────────────
            //                  TELEMETRY
            // ───────────────────────────────────────────
            telemetry.addLine("=== BALL ===");
            telemetry.addData("Found", ball.found);
            if (ball.found) {
                telemetry.addData("Color", ball.color);
                telemetry.addData("Center", "%.1f , %.1f", ball.cx, ball.cy);
                telemetry.addData("Radius", "%.1f", ball.r);
            }

            telemetry.addLine();
            telemetry.addLine("=== APRILTAG ===");
            if (tag != null) {
                telemetry.addData("ID", tag.id);
                telemetry.addData("Center", "%.1f , %.1f", tag.center.x, tag.center.y);
            } else {
                telemetry.addData("Tag", "NONE");
            }

            telemetry.addLine();
            telemetry.addLine("=== HSV CENTER ===");
            if (hsv != null)
                telemetry.addData("H,S,V", "%.0f  %.0f  %.0f", hsv[0], hsv[1], hsv[2]);

            telemetry.update();
        }

        visionPortal.close();
    }


    // =====================================================================
    //       BALL DETECTION PROCESSOR
    // =====================================================================
    public static class BallDetectionProcessor implements VisionProcessor {

        private final Object syncBall = new Object();
        private volatile BallDetection lastBall = new BallDetection(false,0,0,0,"");
        private volatile double[] lastHSV = null;

        private Scalar lowHSV_green = new Scalar(30, 60, 60);
        private Scalar highHSV_green = new Scalar(95, 255, 255);

        private Scalar lowHSV_purple = new Scalar(110, 70, 50);
        private Scalar highHSV_purple = new Scalar(179, 255, 255);

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Not needed for this processor
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Mat copy = frame.clone();

            Imgproc.GaussianBlur(copy, copy, new Size(3, 3), 0);
            Mat hsv = new Mat();
            Imgproc.cvtColor(copy, hsv, Imgproc.COLOR_BGR2HSV);
            Core.normalize(hsv, hsv, 0, 255, Core.NORM_MINMAX);

            Mat maskG = new Mat();
            Mat maskP = new Mat();
            Core.inRange(hsv, lowHSV_green, highHSV_green, maskG);
            Core.inRange(hsv, lowHSV_purple, highHSV_purple, maskP);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
            Imgproc.morphologyEx(maskG, maskG, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskG, maskG, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(maskP, maskP, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskP, maskP, Imgproc.MORPH_CLOSE, kernel);

            boolean found = false;
            double bestR = 0;
            Point bestC = new Point();
            Scalar bestColor = new Scalar(255,255,255);
            String bestName = "";

            List<Mat> masks = List.of(maskG, maskP);
            List<Scalar> colors = List.of(new Scalar(0,255,0), new Scalar(255,0,255));
            List<String> names = List.of("GREEN", "PURPLE");

            for (int i=0; i<masks.size(); i++) {
                List<MatOfPoint> contours = new ArrayList<>();
                Imgproc.findContours(masks.get(i), contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                for (MatOfPoint cnt : contours) {
                    double area = Imgproc.contourArea(cnt);
                    if (area < 150) continue;

                    MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
                    float[] r = new float[1];
                    Point c = new Point();
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

            if (found) {
                Imgproc.circle(frame, bestC, (int) bestR, bestColor, 3);
                Imgproc.putText(frame, bestName, new Point(bestC.x - 30, bestC.y - bestR - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, bestColor, 2);
                synchronized (syncBall) {
                    lastBall = new BallDetection(true, bestC.x, bestC.y, bestR, bestName);
                }
            } else {
                synchronized (syncBall) {
                    lastBall = new BallDetection(false,0,0,0,"");
                }
            }

            double[] hsvC = hsv.get(hsv.rows()/2, hsv.cols()/2);
            if (hsvC != null) lastHSV = hsvC;

            hsv.release();
            maskG.release();
            maskP.release();
            copy.release();

            return frame; // Return the frame to be drawn on the screen
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // Not needed for this processor
        }

        // Getters
        public BallDetection getLastBall() { return lastBall; }
        public double[] getLastHSV() { return lastHSV; }


        // STRUCT
        public static class BallDetection {
            public final boolean found;
            public final double cx, cy, r;
            public final String color;

            public BallDetection(boolean f, double x, double y, double radius, String c) {
                found = f; cx = x; cy = y; r = radius; color = c;
            }
        }
    }
}
