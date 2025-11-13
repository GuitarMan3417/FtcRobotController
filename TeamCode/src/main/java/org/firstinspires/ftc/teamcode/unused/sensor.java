package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "sensor", group = "Sensor")
@Disabled
public class sensor extends LinearOpMode {

    // --- ประกาศอุปกรณ์ ---
    private Rev2mDistanceSensor distanceSensor;
    private Servo SVR_L0;
    private Servo SVR_L1;

    // --- กำหนดระยะตรวจจับลูกบอล (หน่วย: เซนติเมตร) ---
    private static final double BALL_DETECT_DISTANCE_CM = 25.0;

    // --- กำหนดตำแหน่งเซอร์โว ---
    private static final double SERVO_POS_0 = 0.0;     // 0°
    private static final double SERVO_POS_90 = 0.5;    // 90°
    private static final double SERVO_POS_180 = 1.0;   // 180°

    // --- ตัวแปรช่วยป้องกันการทำซ้ำ ---
    private boolean ballDetected = false;

    // --- ตัวแปรแสดงสถานะเซอร์โว ---
    private String servoStatus = "Ready";

    @Override
    public void runOpMode() {

        // --- Map อุปกรณ์กับชื่อใน Configuration ---
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");

        // --- ตั้งค่าเริ่มต้นของเซอร์โว ---
        SVR_L0.setPosition(SERVO_POS_90);
        SVR_L1.setPosition(SERVO_POS_90);

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

                    // เซอร์โว L0 หมุนลง 180°, L1 หมุนขึ้น 0°
                    servoStatus = "L0 Down / L1 Up";
                    SVR_L0.setPosition(SERVO_POS_180);
                    SVR_L1.setPosition(SERVO_POS_0);
                    telemetry.addData("Servo Status", servoStatus);
                    telemetry.update();
                    sleep(1000); // พัก 1 วิ

                    // หมุนกลับตำแหน่งเริ่มต้น 90° ทั้งคู่
                    servoStatus = "Returning to 90°";
                    SVR_L0.setPosition(SERVO_POS_90);
                    SVR_L1.setPosition(SERVO_POS_90);
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
