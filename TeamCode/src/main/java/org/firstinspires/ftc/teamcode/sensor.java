package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "sensor_servo", group = "Sensor")
public class sensor extends LinearOpMode {

    // --- ประกาศอุปกรณ์ ---
    private Rev2mDistanceSensor distanceSensor;
    private Servo servo;

    // --- กำหนดระยะตรวจจับลูกบอล (หน่วย: เซนติเมตร) ---
    private static final double BALL_DETECT_DISTANCE_CM = 25.0;

    // --- กำหนดตำแหน่งเซอร์โว ---
    private static final double SERVO_POS_90 = 0.5;   // ประมาณ 90°
    private static final double SERVO_POS_180 = 1.0;  // ประมาณ 180°

    // --- ตัวแปรช่วยป้องกันการทำซ้ำ ---
    private boolean ballDetected = false;

    // --- ตัวแปรแสดงสถานะเซอร์โว ---
    private String servoStatus = "Ready";

    @Override
    public void runOpMode() {

        // --- Map อุปกรณ์กับชื่อใน Configuration ---
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        servo = hardwareMap.get(Servo.class, "servohitech");

        // --- ตั้งค่าเริ่มต้นของเซอร์โวที่ 90 องศา ---
        servo.setPosition(SERVO_POS_90);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            if (distance > 0 && distance < BALL_DETECT_DISTANCE_CM) {
                telemetry.addData("Ball Status", "✅ Ball Detected!");

                // ทำงานเมื่อเจอลูกบอลครั้งแรก
                if (!ballDetected) {
                    ballDetected = true;

                    // หมุนเซอร์โวลง 180 องศา
                    servoStatus = "Moving Down (180°)";
                    servo.setPosition(SERVO_POS_180);
                    telemetry.addData("Servo Status", servoStatus);
                    telemetry.update();
                    sleep(1000); // พัก 1 วิ

                    // หมุนกลับ 90 องศา
                    servoStatus = "Returning to 90°";
                    servo.setPosition(SERVO_POS_90);
                }

            } else {
                telemetry.addData("Ball Status", "❌ No Ball");
                ballDetected = false;
                servoStatus = "Waiting for Ball";
            }

            // --- แสดงผลสถานะทั้งหมด ---
            telemetry.addData("Servo Status", servoStatus);
            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.update();

            sleep(50);
        }
    }
}
