package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name="TeleOpMode", group="TeleOp")
public class TeleOpMode extends LinearOpMode {
    private DcMotor M_LR;
    private DcMotor M_RR;
    private DcMotor M_LF;
    private DcMotor M_RF;
    @Override
    public void runOpMode(){
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        float spd_F;
        float spd_R;
        float turn;
        float slide;
        telemetry.addLine("System Initialized!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            spd_F = gamepad1.right_trigger;
            spd_R = gamepad1.left_trigger;
            turn = gamepad1.right_stick_x;
            slide = gamepad1.left_stick_x;
            M_LR.setPower(((spd_F - spd_R)*-1) - slide);
            M_RR.setPower((spd_F - spd_R) - slide);
            M_LF.setPower(((spd_F - spd_R)*-1)-turn - slide);
            M_RF.setPower((spd_F - spd_R)-turn - slide);
            telemetry.addData("gamepadRT", gamepad1.right_trigger);
            telemetry.addData("gamepadRStickX", gamepad1.right_stick_x);
            telemetry.addData("M_LR", M_LR.getPower());
            telemetry.addData("M_RR", M_RR.getPower());
            telemetry.addData("M_LF", M_LF.getPower());
            telemetry.addData("M_RF", M_RF.getPower());
            telemetry.update();
     }
    }

}
