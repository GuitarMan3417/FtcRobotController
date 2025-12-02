package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ServoDegreeControl")
public class ServoTest extends LinearOpMode {

    private Servo SVR_L0;  // Servo ซ้าย
    private Servo SVR_L1;  // Servo ขวา (ต้องกลับด้าน)

    // ---- กำหนดองศาที่ต้องการ ---- //
    double upDegree = 150;     // องศาเมื่อกด L1 (ขึ้น)
    double downDegree = 30;    // องศาเมื่อกด R1 (ลง)

    // ฟังก์ชันแปลง องศา → ค่า 0.0–1.0
    private double degToPos(double degree) {
        return Range.clip(degree / 180.0, 0.0, 1.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Map Servo
        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");  // ซ้าย
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");  // ขวา

        // ❌ ไม่ตั้งค่าเริ่มต้น ให้ servo อยู่ที่เดิมตอนเริ่ม run

        waitForStart();

        while (opModeIsActive()) {

            // ➤ กด L1 → Servo ขึ้นพร้อมกัน
            if (gamepad1.left_bumper) {
                double posL0 = degToPos(upDegree);
                double posL1 = 1 - degToPos(upDegree);
                SVR_L0.setPosition(posL0);
                SVR_L1.setPosition(posL1);
            }

            // ➤ กด R1 → Servo ลงพร้อมกัน
            if (gamepad1.right_bumper) {
                double posL0 = degToPos(downDegree);
                double posL1 = 1 - degToPos(downDegree);
                SVR_L0.setPosition(posL0);
                SVR_L1.setPosition(posL1);
            }

            telemetry.addData("SVR_L0 Position", SVR_L0.getPosition());
            telemetry.addData("SVR_L1 Position", SVR_L1.getPosition());
            telemetry.update();
        }
    }
}
