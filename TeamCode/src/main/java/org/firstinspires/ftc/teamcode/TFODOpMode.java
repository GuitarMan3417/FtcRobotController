package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@TeleOp(name="TFOD + Webcam", group="TFOD")
public class TFODOpMode extends LinearOpMode {

    // Motors
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    // Vuforia & TFOD
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // EasyOpenCV
    private OpenCvCamera webcam;

    private static final String VUFORIA_KEY = "Af3/Ogr/////AAABmRAA2ZX9GEBDksJ3SxKLuwRzW/mWhfLzeXA1CBNTwVmGyqgl8Ew1kHiG16VinzAr6dureIB0J7v5T8vQLkGWrG+VDF8r0knHJB9U6MkczWOwODfDwnwtHPPCiJcqL5y+0SCZn4sBX65h1ZVVmFvdNswsHgBVreDMjLwpoXc5k51zK9iP5TXj+Bvefx21gF6NFDw0RsFWscBULKwRyxKFFx0RlZAvalV9LuC0BLQPVknWtSJz0Ijo5P41wquS7nODyhbm4luBOR4e8h2NBiPt6jpqheIQJ8++bi8SqqI1XxP5LeR7cllqdZVYs+z7NJwa4rGvjSsw//Su3QQ7oclR8jilrtp/G7siaAK2PztXf6pM";
    private static final String TFOD_MODEL_ASSET = "model.tflite";
    private static final String[] LABELS = {
            "Class1" // แก้ตาม labels.txt
    };

    @Override
    public void runOpMode() {

        // Init motors
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        leftRear  = hardwareMap.get(DcMotor.class, "LR");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        rightRear  = hardwareMap.get(DcMotor.class, "RR");

        // Vuforia init
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // TFOD init
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        // EasyOpenCV init
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam.openCameraDevice();
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            tfod.activate(); // เริ่ม TFOD

            while (opModeIsActive()) {
                // อ่าน TFOD
                if (tfod != null) {
                    List<Recognition> recognitions = tfod.getUpdatedRecognitions();
                    if (recognitions != null) {
                        telemetry.addData("# Objects Detected", recognitions.size());
                        for (Recognition r : recognitions) {
                            // แสดงข้อมูลแต่ละ object
                            telemetry.addData("Label", r.getLabel());
                            telemetry.addData("Confidence", "%.2f", r.getConfidence());
                            telemetry.addData("Left", r.getLeft());
                            telemetry.addData("Top", r.getTop());
                            telemetry.addData("Right", r.getRight());
                            telemetry.addData("Bottom", r.getBottom());

                            // Example: เคลื่อนหาวัตถุ
                            double centerX = (r.getLeft() + r.getRight()) / 2.0;
                            double error = centerX - 320; // screen width 640
                            double power = error / 320.0;
                            power = Math.max(-1.0, Math.min(1.0, power)); // clamp -1 ถึง 1

                            leftFront.setPower(-power);
                            leftRear.setPower(-power);
                            rightFront.setPower(power);
                            rightRear.setPower(power);
                        }
                        telemetry.update();
                    }
                }
            }
        }

        // Shutdown TFOD
        if (tfod != null) tfod.shutdown();

        // Close webcam
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}
