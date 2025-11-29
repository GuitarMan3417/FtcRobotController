package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
@Disabled

@Autonomous(name="OpenCVBallDetection")
public class OpenCVBallDetection extends LinearOpMode {

    OpenCvWebcam webcam;
    BallPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new BallPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();

        while(opModeIsActive()) {
            // Center X,Y และ radius ของลูกบอล
            telemetry.addData("Ball X", pipeline.ballX);
            telemetry.addData("Ball Y", pipeline.ballY);
            telemetry.addData("Ball Radius", pipeline.ballRadius);
            telemetry.update();

            // ตัวอย่างบังคับหุ่นยนต์
            if(pipeline.ballRadius > 0) { // พบลูกบอล
                if(pipeline.ballX < 140) { // ลูกบอลอยู่ซ้าย
                    // turnLeft();
                } else if(pipeline.ballX > 180) { // ลูกบอลอยู่ขวา
                    // turnRight();
                } else {
                    // moveForward();
                }
            }
        }
    }

    class BallPipeline extends OpenCvPipeline {
        public int ballX = -1;
        public int ballY = -1;
        public int ballRadius = -1;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();

            // แปลง BGR -> HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            // กำหนดช่วงสีลูกบอล (เช่น สีแดง)
            Scalar lowerRed = new Scalar(0, 120, 70);
            Scalar upperRed = new Scalar(10, 255, 255);
            Core.inRange(hsv, lowerRed, upperRed, mask);

            // หาคอนทัวร์
            java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            ballRadius = -1;
            ballX = -1;
            ballY = -1;

            for (MatOfPoint contour : contours) {
                Point center = new Point();
                float[] radius = new float[1];
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                Imgproc.minEnclosingCircle(contour2f, center, radius);

                if(radius[0] > 5) { // กรอง noise เล็กๆ
                    ballX = (int) center.x;
                    ballY = (int) center.y;
                    ballRadius = (int) radius[0];

                    // วาดวงกลมบนหน้ากาก
                    Imgproc.circle(input, center, (int) radius[0], new Scalar(0,255,0), 2);
                }
            }

            return input;
        }
    }
}
