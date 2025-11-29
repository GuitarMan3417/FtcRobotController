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
            .mass(7);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.45)
            .rightFrontMotorName("M_RF")
            .rightRearMotorName("M_RR")
            .leftRearMotorName("M_LR")
            .leftFrontMotorName("M_LF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100,1,1);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .forwardTicksToInches(0.01175) //Insert ticks (push robot 2 inches) 0.0083 0.0083535207 0.0091492 0.01185
            .strafeTicksToInches(0.01)//Insert ticks (push robot 2 inches)0.0022 0.0092
            .turnTicksToInches(0.022)  //Insert ticks (rotate robot full counterclockwise 1 time)0.01795 0.022
            .robotLength(8)//Inch Unit (Needs Adjustment)
            .robotWidth(13)//Inch Unit (Prev 3.5) (Needs Adjustment)
            .rightFrontMotorName("M_RF")
            .rightRearMotorName("M_RR")
            .leftRearMotorName("M_LR")
            .leftFrontMotorName("M_LF")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.REVERSE)
            .rightRearEncoderDirection(Encoder.REVERSE);
    public static Follower createFollower(HardwareMap hardwareMap){
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .driveEncoderLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                /* other builder steps */
                .build();
    }
}
