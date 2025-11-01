package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name="ShootingMechanism" ,group = "Inspection")
public class ShootingMechanism extends LinearOpMode {
    DcMotor M_LF, M_RF, M_LR, M_RR; // Driving DC Motor Declaration
    DcMotor M_S0, M_S1, M_AIN; //Mechanism DC Motor Declaration
    Servo svr_Adj, svr_L0, svr_L1, svr_P; //Servo Declaration


    public void runOpMode(){
        //Define Variables (Min/Max Angle) (Degrees Unit)
        double svrAdjMin = 0;
        double svrAdjMax = 160;
        double svrLMin = 0;
        double svrLMax = 180;
        double svrPMin = 0;
        double svrPMax = 180;

        //Define Variables (Min/Max Power value)
        double M_AINMin = 0.12;
        double M_AINMax = 0.5;
        double M_SMin = 0.1;
        double M_SMax = 1;

        //Artifact Angle Adjustment (Degrees unit)
        double A_position0 = 0;
        double A_position1 = 90;
        double A_position2 = 180;
        
        //Resting Angle Adjustment (Degrees Unit)
        double L_Position0 = 0;
        double L_Position1 = 90;
        double L_Position2 = 180;
        //Define Components
        M_LF = hardwareMap.get(DcMotor.class, "M_LF");
        M_RF = hardwareMap.get(DcMotor.class, "M_RF");
        M_LR = hardwareMap.get(DcMotor.class, "M_LR");
        M_RR = hardwareMap.get(DcMotor.class, "M_RR");
        M_S0 = hardwareMap.get(DcMotor.class, "M_S0");
        M_S1 = hardwareMap.get(DcMotor.class, "M_S1");
        M_AIN = hardwareMap.get(DcMotor.class, "M_AIN");
        svr_Adj = hardwareMap.get(Servo.class, "svr_Adj");
        svr_L0 = hardwareMap.get(Servo.class, "svr_L0");
        svr_L1 = hardwareMap.get(Servo.class, "svr_L1");
        svr_P = hardwareMap.get(Servo.class, "svr_P");
        //Initialize Components
        M_S0.setDirection(DcMotorSimple.Direction.FORWARD);
        M_S1.setDirection(DcMotorSimple.Direction.REVERSE);
        M_AIN.setDirection(DcMotorSimple.Direction.FORWARD);
        svr_Adj.setPosition(svrAdjMin/180);
        svr_L0.setPosition(svrLMin/180);
        svr_L1.setPosition(svrLMin/180);
        svr_P.setPosition(svrPMin/180);
        telemetry.addLine("Init Complete!");
        waitForStart();
        while(opModeIsActive()){
            //Artifact to RestPoint0
            if(gamepad2.right_trigger > 0.8){
                M_AIN.setPower(M_AINMax);
            }
            else{
                M_AIN.setPower(M_AINMin);
            }
            //RestPoint0 to RestPoint1
            if(gamepad2.a){
                //Shooting part (RestPoint1 to Shoot)

            }
            //Override (Manual)
            if(gamepad2.x){
                svr_P.setPosition(A_position0/180);
            }
            if(gamepad2.y){
                svr_P.setPosition(A_position1/180);
            }
            if(gamepad2.b){
                svr_P.setPosition(A_position2/180);
            }
            if(gamepad2.right_bumper){
                svr_L0.setPosition(L_Position1/180);
                svr_L1.setPosition(L_Position1/180);
            }
            if(gamepad2.right_trigger > 0.8){
                svr_L0.setPosition(L_Position0/180);
                svr_L0.setPosition(L_Position0/180);
            }
            if(gamepad2.left_bumper){
                svr_Adj.setPosition(svrAdjMax/180);
            }
            if(gamepad2.left_trigger > 0.8){
                svr_Adj.setPosition(svrAdjMin/180);
            }


            telemetry.addLine("System Inspection");
            telemetry.addLine("Motor Details");
            telemetry.addData("M_S0", M_S0.getPower());
            telemetry.addData("M_S1", M_S1.getPower());
            telemetry.addData("M_AIN", M_AIN.getPower());
            telemetry.addLine("Servo Details");
            telemetry.addData("svr_Adj", svr_Adj.getPosition());
            telemetry.addData("svr_L0", svr_L0.getPosition());
            telemetry.addData("svr_L1", svr_L1.getPosition());
            telemetry.addData("svr_P", svr_P.getPosition());
            telemetry.update();
        }
    }

}
