package org.firstinspires.ftc.teamcode;

// นำเข้าไลบรารีและคลาสที่จำเป็น
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// กำหนดให้คลาสนี้เป็น Autonomous OpMode และตั้งชื่อที่จะแสดงใน Driver Station
@Autonomous(name="DECODE_Autonomous_Mecanum_12_1", group="FTC_DECODE")
public class FtcDecodeAutonomous extends LinearOpMode {

    // ----------------------------------------------------
    // 1. การประกาศตัวแปรฮาร์ดแวร์ (Hardware Declaration)
    // ----------------------------------------------------

    // ประกาศมอเตอร์ขับเคลื่อน 4 ตัว
    private DcMotor M_LF = null;
    private DcMotor M_RF = null;
    private DcMotor M_LR = null;
    private DcMotor M_RR = null;

    // ตัวแปรสำหรับจับเวลา
    private ElapsedTime runtime = new ElapsedTime();

    // ตัวแปรคงที่ (Constants) สำหรับการคำนวณระยะทางด้วย Encoder
    // *** ค่าเหล่านี้ถูกปรับตามข้อมูล 12:1 Gear Ratio และล้อ 104mm ***
    // *** หากหุ่นยนต์ขับไม่ตรง ต้องมาปรับแก้ค่าในส่วนนี้ ***

    // (1) Ticks ต่อรอบของเพลาเอาต์พุต: 
    //    112 Ticks/Motor Rev * 12:1 Gear Ratio = 1344 Ticks
    static final double     COUNTS_PER_MOTOR_REV    = 1344.0;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;      // อัตราทดของชุดเกียร์ขับเคลื่อน (1.0 = ไม่มีการทดเพิ่มเติม)

    // (2) เส้นผ่านศูนย์กลางล้อ 104 mm แปลงเป็นนิ้ว: 104 / 25.4 ≈ 4.0945 นิ้ว
    static final double     WHEEL_DIAMETER_INCHES   = 4.0945;

    // (3) Ticks ต่อ 1 นิ้ว: (Ticks/Rev * Gear Reduction) / (Wheel Circumference)
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    // ค่าที่คำนวณได้โดยประมาณ: 1344 / 12.86 = 104.5

    // ตัวแปรคงที่สำหรับคำนวณการหมุน (Turn)
    // *** ต้องปรับค่านี้หลังจากการทดสอบการหมุน (Tuning) ***
    static final double COUNTS_PER_DEGREE = 25.0; // ค่าสมมติ: ต้องทดสอบว่าหมุน 1 องศา ใช้กี่ Ticks

    // ----------------------------------------------------
    // 2. ฟังก์ชันหลัก (Main Execution)
    // ----------------------------------------------------

    @Override
    public void runOpMode() {

        // ส่งข้อความเริ่มต้นไปยัง Driver Station
        telemetry.addData("Status", "Initialized and Ready to Run. COUNTS_PER_INCH: %.2f", COUNTS_PER_INCH);
        telemetry.update();

        // ----------------------------------------------------
        // 2.1 การตั้งค่าฮาร์ดแวร์ (Hardware Initialization)
        // ----------------------------------------------------

        // ค้นหาและกำหนดค่ามอเตอร์ตามชื่อที่ตั้งในไฟล์ Configuration ของ Robot
        M_LF   = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF  = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR    = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR   = hardwareMap.get(DcMotor.class, "M_RR");

        // กำหนดทิศทางการหมุนของมอเตอร์ (สำหรับ Mecanum Drive)
        M_LF.setDirection(DcMotor.Direction.REVERSE);
        M_LR.setDirection(DcMotor.Direction.REVERSE);
        M_RF.setDirection(DcMotor.Direction.FORWARD);
        M_RR.setDirection(DcMotor.Direction.FORWARD);

        // ตั้งค่าให้มอเตอร์หยุดทันทีเมื่อไม่มีการสั่งงาน (Brake)
        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // รีเซ็ต Encoders ทั้งหมดกลับไปที่ตำแหน่งศูนย์ (0)
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ตั้งค่ามอเตอร์ทั้งหมดให้ใช้โหมด RUN_USING_ENCODERS 
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ----------------------------------------------------
        // 2.2 รอเริ่มต้น (Wait for Start)
        // ----------------------------------------------------
        waitForStart();
        runtime.reset(); // รีเซ็ตตัวจับเวลา

        // ----------------------------------------------------
        // 2.3 การตรวจจับตำแหน่ง Spike Mark ด้วย Vision (Placeholder)
        // ----------------------------------------------------

        // นี่คือตัวแปรสำหรับเก็บผลลัพธ์: 0=LEFT, 1=CENTER, 2=RIGHT
        int spikeMarkPosition = 1;
        // >>>>>>>>>>>>> ใส่โค้ด Vision ของคุณที่นี่ <<<<<<<<<<<<<<
        // ตัวอย่าง: spikeMarkPosition = YourVisionClass.detectPosition(telemetry);

        telemetry.addData("Spike Mark Position", spikeMarkPosition == 0 ? "LEFT" : spikeMarkPosition == 1 ? "CENTER" : "RIGHT");
        telemetry.update();

        // ----------------------------------------------------
        // 2.4 ลำดับการทำงานของ Autonomous (Autonomous Sequence)
        // ----------------------------------------------------

        // ขั้นตอนที่ 1: ขับตรงไปยังพื้นที่ Spike Mark
        driveStraight(28.0, 0.5); // ขับตรงไปข้างหน้า 28 นิ้ว (ปรับระยะตามจริง)

        // ขั้นตอนที่ 2: เคลื่อนที่ตามตำแหน่งที่ Vision ตรวจจับได้
        if (spikeMarkPosition == 0) { // LEFT
            strafeLeft(6.0, 0.4); // Strafe ซ้ายเพื่อวาง Game Element
            // dropElement(); 
            strafeRight(6.0, 0.4); // กลับเข้าที่เดิม
        } else if (spikeMarkPosition == 1) { // CENTER
            // dropElement(); // วางที่ตำแหน่งตรงกลาง
        } else { // RIGHT (spikeMarkPosition == 2)
            strafeRight(6.0, 0.4); // Strafe ขวาเพื่อวาง Game Element
            // dropElement(); 
            strafeLeft(6.0, 0.4); // กลับเข้าที่เดิม
        }

        // ขั้นตอนที่ 3: เดินหน้า/ถอยหลัง เพื่อทำภารกิจต่อไป (เช่น ไป Backdrop)
        driveStraight(-4.0, 0.4); // ถอยหลัง 4 นิ้ว
        turnToHeading(90, 0.5);   // หมุน 90 องศา (ควรใช้ IMU เพื่อความแม่นยำ)
        driveStraight(60.0, 0.7); // ขับตามยาวสนาม

        // ขั้นตอนที่ 4: ทำคะแนนที่ Backdrop และ Park
        turnToHeading(0, 0.5);
        strafeRight(10.0, 0.5);
        driveStraight(6.0, 0.4);
        // scoreBackdrop(); 
        driveStraight(-10.0, 0.5); // Parking

        // แจ้งสถานะการทำงานเสร็จสิ้น
        telemetry.addData("Status", "Autonomous Sequence Complete");
        telemetry.update();
        sleep(1000);
    }

    // ----------------------------------------------------
    // 3. ฟังก์ชันช่วยสำหรับการขับเคลื่อน (Helper Methods)
    // ----------------------------------------------------

    /**
     * ขับหุ่นยนต์ตรงไปข้างหน้าหรือถอยหลังตามระยะทางที่กำหนดโดยใช้ Encoder
     * @param distanceInches ระยะทางเป็นนิ้ว (ค่าบวก = เดินหน้า, ค่าลบ = ถอยหลัง)
     * @param speed กำลังมอเตอร์ (0.0 ถึง 1.0)
     */
    public void driveStraight(double distanceInches, double speed) {
        if (opModeIsActive()) {

            // 1. คำนวณจำนวน Ticks เป้าหมายที่ต้องเคลื่อนที่
            int targetTick = (int)(distanceInches * COUNTS_PER_INCH);

            // 2. กำหนดตำแหน่งเป้าหมายใหม่ (Target Position)
            M_LF.setTargetPosition(M_LF.getCurrentPosition() + targetTick);
            M_RF.setTargetPosition(M_RF.getCurrentPosition() + targetTick);
            M_LR.setTargetPosition(M_LR.getCurrentPosition() + targetTick);
            M_RR.setTargetPosition(M_RR.getCurrentPosition() + targetTick);

            // 3. ตั้งค่าโหมดมอเตอร์เป็น RUN_TO_POSITION
            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // 4. กำหนดกำลังมอเตอร์
            M_LF.setPower(Math.abs(speed));
            M_RF.setPower(Math.abs(speed));
            M_LR.setPower(Math.abs(speed));
            M_RR.setPower(Math.abs(speed));

            // 5. รอจนกว่ามอเตอร์จะถึงตำแหน่งเป้าหมาย
            while (opModeIsActive() &&
                    (M_LF.isBusy() || M_RF.isBusy() || M_LR.isBusy() || M_RR.isBusy())) {

                // แสดงข้อมูล Encoder บน Driver Station
                telemetry.addData("Path",  "Running to FL:%7d FR:%7d",
                        M_LF.getTargetPosition(), M_RF.getTargetPosition());
                telemetry.addData("Current",  "Running at FL:%7d FR:%7d",
                        M_LF.getCurrentPosition(), M_RF.getCurrentPosition());
                telemetry.update();
            }

            // 6. หยุดมอเตอร์และรีเซ็ตโหมด
            stopAllMotors();
            setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * ฟังก์ชันสำหรับการเคลื่อนที่ด้านข้าง (Strafe) ด้วย Mecanum Wheel
     * @param distanceInches ระยะทางเป็นนิ้ว (ค่าบวก = Strafe ขวา, ค่าลบ = Strafe ซ้าย)
     * @param speed กำลังมอเตอร์ (0.0 ถึง 1.0)
     */
    public void strafe(double distanceInches, double speed) {
        // หลักการ Strafe: มอเตอร์ที่อยู่ตรงข้ามกันจะหมุนไปในทิศทางเดียวกัน
        // Strafe ขวา: FL (+), FR (-), BL (-), BR (+)

        if (opModeIsActive()) {

            int targetTick = (int)(distanceInches * COUNTS_PER_INCH);

            // 1. กำหนดตำแหน่งเป้าหมายใหม่ (Target Position) 
            //    (targetTick เป็นบวกสำหรับการ Strafe ขวา)
            M_LF.setTargetPosition(M_LF.getCurrentPosition() + targetTick);
            M_RF.setTargetPosition(M_RF.getCurrentPosition() - targetTick);
            M_LR.setTargetPosition(M_LR.getCurrentPosition() - targetTick);
            M_RR.setTargetPosition(M_RR.getCurrentPosition() + targetTick);

            // 2. เข้าสู่โหมด RUN_TO_POSITION และกำหนดกำลัง
            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            M_LF.setPower(Math.abs(speed));
            M_RF.setPower(Math.abs(speed));
            M_LR.setPower(Math.abs(speed));
            M_RR.setPower(Math.abs(speed));

            // 3. รอจนกว่ามอเตอร์จะถึงตำแหน่งเป้าหมาย
            while (opModeIsActive() &&
                    (M_LF.isBusy() || M_RF.isBusy() || M_LR.isBusy() || M_RR.isBusy())) {
                telemetry.addData("Path", "Strafe Running...");
                telemetry.update();
            }

            // 4. หยุดมอเตอร์และรีเซ็ตโหมด
            stopAllMotors();
            setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // ฟังก์ชันย่อสำหรับ Strafe ซ้าย
    public void strafeLeft(double distanceInches, double speed) {
        strafe(-distanceInches, speed); // ใช้ค่าลบเพื่อ Strafe ซ้าย
    }

    // ฟังก์ชันย่อสำหรับ Strafe ขวา
    public void strafeRight(double distanceInches, double speed) {
        strafe(distanceInches, speed); // ใช้ค่าบวกเพื่อ Strafe ขวา
    }

    /**
     * หมุนหุ่นยนต์ตามองศาที่กำหนดโดยใช้ Encoder (ไม่มี Gyro)
     * *** ความแม่นยำจะต่ำกว่าการใช้ Gyroscope/IMU ***
     * @param degrees องศาที่ต้องการหมุน (ค่าบวก = หมุนขวา, ค่าลบ = หมุนซ้าย)
     * @param speed กำลังมอเตอร์
     */
    public void turnToHeading(double degrees, double speed) {

        if (opModeIsActive()) {

            int targetTick = (int)(degrees * COUNTS_PER_DEGREE);

            // หมุนขวา (บวก): FL(+), FR(-), BL(+), BR(-)
            M_LF.setTargetPosition(M_LF.getCurrentPosition() + targetTick);
            M_RF.setTargetPosition(M_RF.getCurrentPosition() - targetTick);
            M_LR.setTargetPosition(M_LR.getCurrentPosition() + targetTick);
            M_RR.setTargetPosition(M_RR.getCurrentPosition() - targetTick);

            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            M_LF.setPower(Math.abs(speed));
            M_RF.setPower(Math.abs(speed));
            M_LR.setPower(Math.abs(speed));
            M_RR.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (M_LF.isBusy() || M_RF.isBusy() || M_LR.isBusy() || M_RR.isBusy())) {
                telemetry.addData("Path", "Turning...");
                telemetry.update();
            }

            stopAllMotors();
            setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * ตั้งค่าโหมดการทำงานของมอเตอร์ขับเคลื่อนทั้งหมด
     */
    private void setDriveMotorMode(DcMotor.RunMode mode) {
        M_LF.setMode(mode);
        M_RF.setMode(mode);
        M_LR.setMode(mode);
        M_RR.setMode(mode);
    }

    /**
     * สั่งหยุดมอเตอร์ขับเคลื่อนทั้งหมด
     */
    private void stopAllMotors() {
        M_LF.setPower(0);
        M_RF.setPower(0);
        M_LR.setPower(0);
        M_RR.setPower(0);
    }
}