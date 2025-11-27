package org.firstinspires.ftc.teamcode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOpFinal extends OpMode {
    //Shooting Variables
    private int shootingAngle = 0; //Initial shooting angle
    private int angleAdd = 1; //Servo angle addition value
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    DcMotor M_AIN, M_S0, M_S1, M_bl, M_LF, M_RF, M_LR, M_RR;
    Servo SVR_L0, SVR_L1;
    //System Timer
    Timer motorTimer = new Timer();
    TimerTask motorAct = new TimerTask() {
        @Override
        public void run() {
            M_bl.setPower(1);
        }
    };


    public void shootingAct(){
        //Angle Adjustment (Shooting)
        if(gamepad2.right_bumper){
            shootingAngle += angleAdd;
        }
        if(gamepad2.left_bumper){
            shootingAngle -= angleAdd;
        }
        shootingAngle = Math.max(0, Math.min(180, shootingAngle));
        SVR_L0.setPosition((double) shootingAngle /180); //Scale Angle from 0-180 to 0-1

        //Intake Motor
        double intakePower = 0;
        if(gamepad1.b){
            intakePower = -0.65;
        }
        else if (gamepad1.left_trigger > 0.1) {
            intakePower = 0.18;  // ดูดช้า (กด L2)
        }
        else if (gamepad1.right_trigger > 0.1) {
            intakePower = 0.55;  // ดูดเร็ว (กด R2)
        }
        M_AIN.setPower(intakePower);

        //Spin-Up
        M_S0.setPower(gamepad2.a ? 0.85:0); //A pressed = spin up
        M_S0.setPower(gamepad2.x ? -1:0); //X pressed = reverse direction with maximum speed

        //Blocking motor & shooting motor
        if(gamepad2.b){
            M_S1.setPower(1);
            motorTimer.schedule(motorAct, 300); //Run this task once within 300 ms
        }
        else{
            M_bl.setPower(0);
            M_S1.setPower(0.35);
        }

    }
    @Override
    public void init() {
        //Declare motor configuration
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN"); //Artifact In
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0"); //Spin up Motor
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1"); //Shoot Motor
        M_bl = hardwareMap.get(DcMotor.class, "M_bl"); //Blocking motor (Core Hex Recommended)
        SVR_L0 = hardwareMap.get(Servo.class, "SVR_L0");
        SVR_L1 = hardwareMap.get(Servo.class, "SVR_L1");

        M_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_AIN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("Pose X", follower.getPose().getX());
        telemetryM.debug("Follower Y", follower.getPose().getY());

    }
}