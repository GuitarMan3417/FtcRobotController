package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants() //Weight Declaration
            .mass(5.1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("M_RF")
            .rightRearMotorName("M_RR")
            .leftRearMotorName("M_LR")
            .leftFrontMotorName("M_LF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100,1,1);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .forwardTicksToInches(0.0023) //Insert ticks (push robot 2 inches) 0.0017
            .strafeTicksToInches(0.0022)//Insert ticks (push robot 2 inches)0.0022
            .turnTicksToInches(0.0034)  //Insert ticks (rotate robot full counterclockwise 1 time)0.0040
            .robotLength(13.5)//Inch Unit
            .robotWidth(9.5)//Inch Unit
            .rightFrontMotorName("M_RF")
            .rightRearMotorName("M_RR")
            .leftRearMotorName("M_LR")
            .leftFrontMotorName("M_LF")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap){
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .driveEncoderLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                /* other builder steps */
                .build();
    }
}
