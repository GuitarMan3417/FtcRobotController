package org.firstinspires.ftc.teamcode.autonomousCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//Intake 1 standby 0.18
//Shoot stby 0.35 shoot 1
@Autonomous(name="AutoArtifact200", group = "Autonomous")
public class AutoArtifact200 extends OpMode {
    DcMotor M_LF, M_RF, M_LR, M_RR;   // มอเตอร์ล้อทั้ง 4 (Mecanum)
    DcMotor M_S0, M_S1, M_bl, M_AIN;

    private int pathState;

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    public void buildPaths(){
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 8.000), new Pose(88.000, 87.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 87.500), new Pose(104.000, 66.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.000, 66.0000), new Pose(130.000, 66.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 59.600), new Pose(88.000, 87.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 87.500), new Pose(104.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.000, 35.000), new Pose(130.000, 35.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 35.000), new Pose(88.000, 87.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }


    public void pathUpdate(){
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

            case 2:     // หยุด 1 วินาที + สั่งให้มอเตอร์ทำงาน
                if(pathTimer.getElapsedTimeSeconds() < 5){
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
                    follower.followPath(Path2);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.2);
                    M_AIN.setPower(1);
                    follower.followPath(Path3);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    M_AIN.setPower(0.18);
                    follower.setMaxPower(0.5);

                    follower.followPath(Path4);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.5);
                    if(pathTimer.getElapsedTimeSeconds() < 5){
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
                        follower.followPath(Path5);
                        setPathState(6);
                    }

                }
                break;

            case 6:
                if(!follower.isBusy()){
                    M_AIN.setPower(1);
                    follower.setMaxPower(0.2);
                    follower.followPath(Path6);
                    setPathState(7);
                }
                break;

            case 7:
                M_AIN.setPower(1);
                if(!follower.isBusy()){
                    M_AIN.setPower(0.18);
                    follower.setMaxPower(0.5);
                    follower.followPath(Path7);
                    setPathState(8);

                }
                break;

            case 8:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds() < 5){
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
    }

    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init(){
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // ===== ระบบเบรกมอเตอร์ =====
        DcMotor M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        DcMotor M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        DcMotor M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        DcMotor M_RR = hardwareMap.get(DcMotor.class, "M_RR");
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");
        M_bl = hardwareMap.get(DcMotor.class, "M_bl");
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");


        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // ===========================

        buildPaths();
        follower.setStartingPose(new Pose(88,8, Math.toRadians(90)));
        follower.setMaxPower(0.5);
    }


    @Override
    public void loop(){
        follower.update();
        pathUpdate();
        telemetry.addData("Path", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getHeading());
    }
}
