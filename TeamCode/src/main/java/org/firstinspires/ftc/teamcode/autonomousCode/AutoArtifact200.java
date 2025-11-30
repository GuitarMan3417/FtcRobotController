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


@Autonomous(name="AutoArtifact200", group = "Autonomous")
public class AutoArtifact200 extends OpMode {
    DcMotor M_LF, M_RF, M_LR, M_RR;   // มอเตอร์ล้อทั้ง 4 (Mecanum)

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
                        new BezierLine(new Pose(88.000, 87.500), new Pose(104.000, 59.600))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.000, 59.600), new Pose(130.000, 59.600))
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
            case 0:
                follower.followPath(Path1);
                pathState++;
                setPathState(pathState);
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(Path2);
                    pathState++;
                    setPathState(pathState);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(Path3);
                    pathState++;
                    setPathState(pathState);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(Path4);
                    pathState++;
                    setPathState(pathState);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(Path5);
                    pathState++;
                    setPathState(pathState);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(Path6);
                    pathState++;
                    setPathState(pathState);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(Path7);
                    pathState++;
                    setPathState(pathState);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    setPathState(-1);
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
