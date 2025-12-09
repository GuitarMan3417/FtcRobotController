package org.firstinspires.ftc.teamcode.autonomousCode;

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

//Intake 1 standby 0.18
//Shoot stby 0.35 shoot 1
@Autonomous(name="AutoArtifactTest: Blue2", group = "Autonomous")
public class AutoArtifactTest300 extends OpMode {
    DcMotor M_LF, M_RF, M_LR, M_RR;   // มอเตอร์ล้อทั้ง 4 (Mecanum)
    DcMotor M_S0, M_S1, M_bl, M_AIN;
    Servo SVR_sw;

    private int pathState;
    private double servoDelay = 4.3; //Delay before servoAct
    private double systemDelay = 5.5; //Path Duration Timer
    private double maxSpeed = 0.65;

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private double maxS1Power = -0.73;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6;


    public void buildPaths(){
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.347, 101.364), new Pose(41.240, 101.794))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.240, 101.794), new Pose(40.488, 86.973))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.488, 86.973), new Pose(9.558, 86.651))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(9.558, 86.651), new Pose(45.965, 97.605))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(135))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.965, 97.605), new Pose(17.183, 111.674))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();
    }


        public void pathUpdate(){
        switch(pathState){
            case 0:     // เริ่ม Path1
                M_S1.setPower(maxS1Power);
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:     // รอ Path1 จบ
                M_S1.setPower(maxS1Power);
                if(!follower.isBusy()){
                    setPathState(2);  // ไปสถานะหยุด 1 วิ
                }
                break;

            case 2:     // หยุด 1 วินาที + สั่งให้มอเตอร์ทำงาน
                if(pathTimer.getElapsedTimeSeconds() < systemDelay){

                    // มอเตอร์ทำงานระหว่างหยุด 1 วิ
                    M_S0.setPower(1.0);
                    M_S1.setPower(maxS1Power);
                    M_bl.setPower(-1.0);
                    if(pathTimer.getElapsedTimeSeconds() > servoDelay){
                        SVR_sw.setPosition(0.5);
                    }
                    M_AIN.setPower(1);
                } else {
                    // ครบ 1 วิแล้วปิดมอเตอร์
                    SVR_sw.setPosition(0);
                    M_S0.setPower(0);
                    M_S1.setPower(-0.35);
                    M_bl.setPower(0);
                    M_AIN.setPower(0.18);



                    // ไป Path2
                    follower.followPath(Path2);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.setMaxPower(maxSpeed - 0.4);
                    M_AIN.setPower(1);
                    follower.followPath(Path3);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    M_AIN.setPower(0.18);
                    follower.setMaxPower(maxSpeed);

                    follower.followPath(Path4);
                    setPathState(5);
                }
                break;


            case 5  :
                follower.setMaxPower(maxSpeed);
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds() < systemDelay){
                        // มอเตอร์ทำงานระหว่างหยุด 1 วิ

                        M_S0.setPower(1.0);
                        M_S1.setPower(maxS1Power);
                        M_bl.setPower(-1.0);
                        if(pathTimer.getElapsedTimeSeconds() > servoDelay){
                            SVR_sw.setPosition(0.5);
                        }
                        M_AIN.setPower(+1);
                    } else {
                        // ครบ 1 วิแล้วปิดมอเตอร์
                        SVR_sw.setPosition(0);
                        M_S0.setPower(0);

                        M_bl.setPower(0);
                        M_AIN.setPower(0.18);
                        follower.followPath(Path5);
                        setPathState(6);  // จบ
                    }
                }
                break;
            case 6:
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
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");
        M_bl = hardwareMap.get(DcMotor.class, "M_bl");
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");
        SVR_sw = hardwareMap.get(Servo.class, "SVR_sw");

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // ===========================

        buildPaths();
        follower.setStartingPose(new Pose(15,128, Math.toRadians(135)));
        follower.setMaxPower(maxSpeed);
    }


    @Override
    public void loop(){
        follower.update();
        pathUpdate();
        telemetry.addData("Path", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
    }
}
