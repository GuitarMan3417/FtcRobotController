package org.firstinspires.ftc.teamcode.autonomousCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous (name="Autonomous-Custom", group="Autonomous")
public class autonomous_Custom extends OpMode {
    private int pathState;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public PathChain Path1, Path2, Path3;


    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.175, 83.682))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.175, 83.682), new Pose(126.269, 83.517))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.269, 83.517), new Pose(126.435, 120.470))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void init(){ //First run

    }
    public void init_loop(){ //Loop (Initialize Mode)

    }
    public void loop(){ //While OpMode is active

    }

    public void start(){ //First run

    }
    public void stop(){

    }
}
