package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import android.graphics.Bitmap;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.tensorflow.lite.Interpreter;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name="OpenCV + TFLite OpMode", group="Vision")
public class OpenCvTFLiteOpMode extends LinearOpMode {

    private Interpreter tflite;
    private List<String> labels = new ArrayList<>();
    private OpenCvCamera camera;

    private final int MODEL_INPUT_SIZE = 224; // เปลี่ยนตาม model ของคุณ
    private final float MIN_CONFIDENCE = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. โหลด TensorFlow Lite Model
        try {
            tflite = new Interpreter(loadModelFile("model.tflite"));
            labels = loadLabels("labels.txt");
        } catch (IOException e) {
            telemetry.addLine("Error loading model: " + e.getMessage());
            telemetry.update();
            return;
        }

        // 2. ตั้งค่า Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                // แปลง Mat -> Bitmap
                Bitmap bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(input, bitmap);

                // Run TFLite
                float[][] output = runTFLite(bitmap);

                // วาด bounding box (ตัวอย่าง fixed box)
                for (int i = 0; i < output.length; i++) {
                    int labelIndex = (int) output[i][0];
                    float confidence = output[i][1];
                    if (confidence > MIN_CONFIDENCE) {
                        Imgproc.rectangle(input, new Point(50,50), new Point(200,200), new Scalar(0,255,0), 3);
                        Imgproc.putText(input, labels.get(labelIndex) + " " + String.format("%.2f", confidence),
                                new Point(50,45), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(0,255,0), 2);
                    }
                }
                return input;
            }
        });

        // ใช้ anonymous class แทน lambda
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Camera ready!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
            sleep(50);
        }

        camera.stopStreaming();
    }

    // แปลง Bitmap -> ByteBuffer สำหรับ TFLite
    private ByteBuffer convertBitmapToByteBuffer(Bitmap bitmap) {
        Bitmap scaled = Bitmap.createScaledBitmap(bitmap, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, true);
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(4 * MODEL_INPUT_SIZE * MODEL_INPUT_SIZE * 3);
        byteBuffer.order(ByteOrder.nativeOrder());

        int[] pixels = new int[MODEL_INPUT_SIZE * MODEL_INPUT_SIZE];
        scaled.getPixels(pixels, 0, MODEL_INPUT_SIZE, 0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);

        for (int pixel : pixels) {
            float r = ((pixel >> 16) & 0xFF) / 255.0f;
            float g = ((pixel >> 8) & 0xFF) / 255.0f;
            float b = (pixel & 0xFF) / 255.0f;
            byteBuffer.putFloat(r);
            byteBuffer.putFloat(g);
            byteBuffer.putFloat(b);
        }
        return byteBuffer;
    }

    // เรียก TFLite
    private float[][] runTFLite(Bitmap bitmap) {
        ByteBuffer input = convertBitmapToByteBuffer(bitmap);
        float[][] output = new float[1][labels.size()]; // สมมติ classification model
        tflite.run(input, output);

        // แปลง output เป็น [labelIndex, confidence]
        float[][] results = new float[labels.size()][2];
        for (int i = 0; i < labels.size(); i++) {
            results[i][0] = i;
            results[i][1] = output[0][i];
        }
        return results;
    }

    // โหลด model.tflite
    private ByteBuffer loadModelFile(String modelPath) throws IOException {
        InputStream is = hardwareMap.appContext.getAssets().open(modelPath);
        byte[] bytes = new byte[is.available()];
        is.read(bytes);
        is.close();
        ByteBuffer buffer = ByteBuffer.allocateDirect(bytes.length);
        buffer.order(ByteOrder.nativeOrder());
        buffer.put(bytes);
        buffer.rewind();
        return buffer;
    }

    // โหลด labels.txt
    private List<String> loadLabels(String filename) {
        List<String> labelList = new ArrayList<>();
        try {
            InputStream is = hardwareMap.appContext.getAssets().open(filename);
            byte[] buffer = new byte[is.available()];
            is.read(buffer);
            is.close();

            String[] lines = new String(buffer).split("\n");
            for (String line : lines) {
                labelList.add(line.trim());
            }
        } catch (IOException e) {
            telemetry.addLine("Error loading labels: " + e.getMessage());
            telemetry.update();
        }
        return labelList;
    }
}
