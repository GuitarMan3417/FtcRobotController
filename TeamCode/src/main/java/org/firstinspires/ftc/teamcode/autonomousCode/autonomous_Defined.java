package org.firstinspires.ftc.teamcode.autonomousCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name="Autonomous: Artifact Catch", group = "Autonomous")
public class autonomous_Defined extends OpMode {
    private int pathState;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;
    public void buildPaths(){
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.175, 87.241))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.175, 87.241), new Pose(52.555, 91.095))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(52.555, 91.095), new Pose(33.635, 59.679))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.635, 59.679), new Pose(14.248, 59.912))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.248, 59.912), new Pose(56.175, 87.358))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.175, 87.358), new Pose(52.555, 91.095))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(52.555, 91.095), new Pose(33.635, 35.387))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.635, 35.387), new Pose(14.949, 35.504))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.949, 35.504), new Pose(56.292, 87.474))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.292, 87.474), new Pose(52.438, 91.212))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(52.438, 91.212), new Pose(9.109, 69.956))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
    public void pathUpdate(){
        switch(pathState){
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(Path2);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(Path3);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(Path4);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(Path5);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(Path6);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(Path7);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(Path8);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(Path9);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    follower.followPath(Path10);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(Path11);
                    setPathState(11);
                }
                break;
            case 11:
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
        buildPaths();
        follower.setStartingPose(new Pose(56,8, Math.toRadians(90)));
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
