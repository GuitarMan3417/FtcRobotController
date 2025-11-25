package org.firstinspires.ftc.teamcode.autonomousCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name="Autonomous", group = "Autonomous")
public class AutoArtifact extends OpMode {
    private int pathState;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;
    public void buildPaths(){
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(60.730, 82.803))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.730, 82.803), new Pose(35.153, 59.679))
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(175))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(35.153, 59.679), new Pose(13.547, 59.912))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.547, 59.912), new Pose(60.730, 82.803))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.730, 82.803), new Pose(35.153, 35.387))
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(175))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(35.153, 35.387), new Pose(14.015, 35.620))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.015, 35.620), new Pose(60.847, 82.803))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.847, 82.803), new Pose(9.343, 69.956))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
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
