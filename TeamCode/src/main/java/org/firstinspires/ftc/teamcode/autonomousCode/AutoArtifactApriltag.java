package org.firstinspires.ftc.teamcode.autonomousCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AUTO_ALIGN_PATH2")
public class AutoArtifactApriltag extends OpMode {

    // --- DRIVE MOTORS ---
    DcMotor M_LF, M_RF, M_LR, M_RR;
    DcMotor M_S0, M_S1, M_bl, M_AIN;

    // --- PEDRO ---
    Follower follower;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;


    // --- STATE ---
    int pathState = 0;
    ElapsedTime pathTimer;

    // -------------------------
    //      APRILTAG CAMERA
    // -------------------------
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    int TAG_ID = 20;                // ⭐ หาแท็ก ID 20
    double CENTER_TOLERANCE = 20;   // px
    double ROTATE_POWER = 0.50;


    @Override
    public void init() {

        // ---------------------------
        // MOTOR
        // ---------------------------
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");
        M_bl = hardwareMap.get(DcMotor.class, "M_bl");
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");

        M_RF.setDirection(DcMotor.Direction.REVERSE);
        M_RR.setDirection(DcMotor.Direction.REVERSE);

        // ---------------------------
        // PEDRO FOLLOWER
        // ---------------------------
        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        // ---------------------------
        // CAMERA + APRILTAG
        // ----------------------------
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        pathTimer = new ElapsedTime();
    }

    public void buildPaths(){
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 87.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 87.500), new Pose(45.000, 66.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(176))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.000, 63.000), new Pose(7.000, 66.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(7.000, 66.000), new Pose(56.000, 87.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 87.500), new Pose(45.000, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(176))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.000, 35.500), new Pose(7.000, 35.500))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(7.000, 35.500), new Pose(56.000, 87.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    // ----------------------------------------------------
    //      FUNCTION: ALIGN TO APRILTAG (ROTATE ONLY)
    // ----------------------------------------------------
    public boolean alignToTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.isEmpty()) {
            stopDrive();
            return false;
        }

        AprilTagDetection tag = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == TAG_ID) {
                tag = detection;
                break;
            }
        }
        if (tag == null) {
            stopDrive();
            return false;
        }

        double error = tag.ftcPose.x;

        if (Math.abs(error) < CENTER_TOLERANCE) {
            stopDrive();
            return true;
        }

        double p = ROTATE_POWER * Math.signum(error);

        M_LF.setPower(p);
        M_LR.setPower(p);
        M_RF.setPower(-p);
        M_RR.setPower(-p);

        return false;
    }


    public void stopDrive() {
        M_LF.setPower(0);
        M_RF.setPower(0);
        M_LR.setPower(0);
        M_RR.setPower(0);
    }
    
    public void setPathState(int state) {
        pathState = state;
        if (pathTimer != null) {
            pathTimer.reset();
        }
    }


    // ----------------------------------------------------
    //                  LOOP STATE MACHINE
    // ----------------------------------------------------
    @Override
    public void loop() {
        switch(pathState){
            case 0:     // เริ่ม Path1
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:     // รอ Path1 จบ
                if(!follower.isBusy()){
                    setPathState(2);  // ไปสถานะหยุด 1 วิ
                }
                break;
            case 2:
                // ⭐ ตอนนี้กำลังเดิน Path2
                // เมื่อถึงตำแหน่งที่ต้องการหมุน → ใส่เงื่อนไขตรงนี้
                if (follower.getPose().getX() > 25) {
                    stopDrive();
                    setPathState(20);   // ไปสเต็ปหมุนหาแท็ก
                }
                break;

            case 20:
                // ⭐ หมุนหา AprilTag ID 20
                if (alignToTag()) {
                    setPathState(4);   // หมุนสำเร็จแล้ว
                }
                break;

            case 4:
                // ⭐ หมุนเสร็จ → ค่อยให้เคลื่อนต่อ Path2
                follower.followPath(Path2);
                setPathState(5);
                break;

            case 5:     // หยุด 1 วินาที + สั่งให้มอเตอร์ทำงาน
                if(pathTimer.seconds() < 5){
                    // มอเตอร์ทำงานระหว่างหยุด 1 วิ
                    M_S0.setPower(1.0);
                    M_S1.setPower(-1.0);
                    M_bl.setPower(-1.0);
                    M_AIN.setPower(1);
                } else {
                    // ครบ 1 วิแล้วปิดมอเตอร์

                    M_S0.setPower(0);
                    M_S1.setPower(0.35);
                    M_bl.setPower(0);
                    M_AIN.setPower(0.18);



                    // ไป Path2
                    follower.followPath(Path3);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.2);
                    M_AIN.setPower(1);
                    follower.followPath(Path4);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()){
                    M_AIN.setPower(0.18);
                    follower.setMaxPower(0.5);

                    follower.followPath(Path5);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.5);
                    if(pathTimer.seconds() < 5){
                        // มอเตอร์ทำงานระหว่างหยุด 1 วิ
                        M_S0.setPower(1.0);
                        M_S1.setPower(-1.0);
                        M_bl.setPower(-1.0);
                        M_AIN.setPower(1);
                    } else {
                        // ครบ 1 วิแล้วปิดมอเตอร์
                        M_S0.setPower(0);
                        M_S1.setPower(0.35);
                        M_bl.setPower(0);
                        M_AIN.setPower(0.18);
                        follower.followPath(Path6);
                        setPathState(9);
                    }

                }
                break;

            case 9:
                if(!follower.isBusy()){
                    M_AIN.setPower(1);
                    follower.setMaxPower(0.2);
                    follower.followPath(Path7);
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy()){
                    setPathState(11);

                }
                break;

            case 11:
                if(!follower.isBusy()){
                    if(pathTimer.seconds() < 5){
                        // มอเตอร์ทำงานระหว่างหยุด 1 วิ
                        M_S0.setPower(1.0);
                        M_S1.setPower(-1.0);
                        M_bl.setPower(-1.0);
                        M_AIN.setPower(1);
                    } else {
                        // ครบ 1 วิแล้วปิดมอเตอร์
                        M_S0.setPower(0);
                        M_S1.setPower(0.35);
                        M_bl.setPower(0);
                        M_AIN.setPower(0.18);
                    }
                    setPathState(-1);  // จบ
                }
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.update();
    }
}
