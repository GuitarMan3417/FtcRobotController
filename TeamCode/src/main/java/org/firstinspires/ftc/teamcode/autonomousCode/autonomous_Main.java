package org.firstinspires.ftc.teamcode.autonomousCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous (name="Autonomous: Main", group="Autonomous")
public class autonomous_Main extends OpMode {
    public int runState = 0;
    //Webcam Declaration
    DcMotor M_AIN, M_S0, M_bl, M_S1;
    Servo SVR_L0, SVR_L1;
    private int pathState;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public PathChain Path1, Path2, Path3;
    Thread shootAct = new Thread(new Runnable() { //Shooting Functions
        @Override
        public void run() {
            telemetry.addLine("Shooting!");
            M_AIN.setPower(0.55);
            M_S1.setPower(-1);
            M_S0.setPower(1);
            try{
                Thread.sleep(600);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            M_bl.setPower(-1);



        }
    });
    void shootMecDisabled(){ //Disabled only upper mechanism
        M_bl.setPower(0);
        M_S1.setPower(-0.35);
    }
    void shootAllDisabled(){ //Disabled All mechanism
        M_AIN.setPower(0);
        M_S0.setPower(0);
        M_bl.setPower(0);
        M_S1.setPower(0);

    }
    Thread shootPrep = new Thread(new Runnable() {
        @Override
        public void run() {
            M_AIN.setPower(0.18);
            M_S1.setPower(-0.35);
        }
    });
    Thread testFunc = new Thread(new Runnable() {
        @Override
        public void run(){
            shootPrep.start();
            try{
               Thread.sleep(5000);
            }
            catch(InterruptedException e){
                throw new RuntimeException(e);
            }
            shootAct.start();
            try{
                Thread.sleep(5000);
            }
            catch(InterruptedException e){
                throw new RuntimeException(e);
            }
            shootMecDisabled();
            try{
                Thread.sleep(5000);
            }
            catch(InterruptedException e){
                throw new RuntimeException(e);
            }
            shootAllDisabled();
            try{
                Thread.sleep(5000);
            }
            catch(InterruptedException e){
                throw new RuntimeException(e);
            }
        }
    });
    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.175, 87.994))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.175, 87.994), new Pose(44.771,59.655))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.771,59.655), new Pose(15.245,59.820))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
    public void pathUpdate(){
         /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
        switch(pathState){
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(Path3, true);
                    setPathState(3);
                }
            case 3:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }



    @Override
    public void init(){ //First run
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        follower.setMaxPower(0.5);
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN"); //Artifact In
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0"); //Spin up Motor
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1"); //Shoot Motor
        M_bl = hardwareMap.get(DcMotor.class, "M_bl"); //Blocking motor (Core Hex Recommended)
        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        M_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void init_loop(){ //Loop (Initialize Mode)

    }
    @Override
    public void loop(){ //While OpMode is active
        /*
        follower.update();
        pathUpdate();
         */
        if(runState == 0){
            testFunc.start();
            runState++;
        }

        telemetry.addData("Path", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void start(){ //First run

    }
    @Override
    public void stop(){ //Unused

    }
}
