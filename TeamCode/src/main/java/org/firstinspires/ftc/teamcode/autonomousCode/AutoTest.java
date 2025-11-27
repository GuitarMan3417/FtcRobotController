package org.firstinspires.ftc.teamcode.autonomousCode;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@SuppressWarnings("unused")
@Autonomous(name="AutoTest", group = "Autonomous")
public class AutoTest extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // ---- เพิ่ม hardware ----
    private DcMotor M_S0, M_S1;
    private Servo SVR_L1;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // ---- map hardware ----
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");

        // reset motor
        M_S0.setPower(0.0);
        M_S1.setPower(0.0);
        SVR_L1.setPosition(0.0);   // ตำแหน่งเริ่มต้น


        buildPaths();
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
    }
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
        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(9.343, 69.956), new Pose(63.63,78.71))
                )
                .setConstantHeadingInterpolation(Math.toRadians(75))
                .build();
    }

    public void pathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;

            // -------------------------
            // **1: Path1 เสร็จ → เริ่มหมุนมอเตอร์**
            // -------------------------
            case 1:
                if(!follower.isBusy()) {
                    // Path1 เสร็จ → หยุดหุ่นยนต์ (follower ไม่เดินต่อ)
                    follower.breakFollowing(); // หยุดการเคลื่อนที่ของ follower

                    // หมุนมอเตอร์
                    M_S0.setPower(0.8);
                    M_S1.setPower(1.0);


                    // ตั้งเวลาเพื่อรอ Servo
                    pathTimer.resetTimer();

                    setPathState(2); // ไป state รอ 1.5 วินาที
                }
                break;

            case 2:
                telemetry.addLine("[STATE 1.5] Waiting before servo...");
                telemetry.addData("Wait Time", pathTimer.getElapsedTime());
                telemetry.addData("M_S0 Power", M_S0.getPower());
                telemetry.addData("M_S1 Power", M_S1.getPower());

                // รอ 1.5 วินาที
                if(pathTimer.getElapsedTime() > 1.5){
                    // ดันเซอร์โว
                    SVR_L1.setPosition(1.0);

                    // เริ่ม Path2
                    follower.followPath(Path2);
                    setPathState(3);
                }
                break;


            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Path3);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path4);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Path5);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Path6);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Path7);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(Path8);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(Path9);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
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
    public void loop(){
        follower.update();
        pathUpdate();

        telemetry.addLine("==== Autonomous Debug ====");
        telemetry.addData("State", pathState);
        telemetry.addData("Path Timer", pathTimer.getElapsedTime());
        telemetry.addData("OpMode Time", opModeTimer.getElapsedTime());

        telemetry.addLine("---- Robot Pose ----");
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading(deg)", "%.2f", Math.toDegrees(follower.getHeading()));

        telemetry.addLine("---- Motor ----");
        telemetry.addData("M_S0 Power", "%.2f", M_S0.getPower());
        telemetry.addData("M_S1 Power", "%.2f", M_S1.getPower());

        telemetry.addLine("---- Servo ----");
        telemetry.addData("SVR_L1 Pos", "%.2f", SVR_L1.getPosition());
        telemetry.update();
    }
}
