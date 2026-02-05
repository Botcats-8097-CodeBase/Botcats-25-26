package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-44.71791)
            .lateralZeroPowerAcceleration(-57.12016656)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0));
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008,0.0,0,0.5,0.0))
//            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotConstants.rightFrontDiveName)
            .rightRearMotorName(RobotConstants.rightBackDiveName)
            .leftRearMotorName(RobotConstants.leftBackDiveName)
            .leftFrontMotorName(RobotConstants.leftFrontDiveName)
            .leftFrontMotorDirection(RobotConstants.leftFrontDiveDirection)
            .leftRearMotorDirection(RobotConstants.leftBackDiveDirection)
            .rightFrontMotorDirection(RobotConstants.rightFrontDiveDirection)
            .rightRearMotorDirection(RobotConstants.rightBackDiveDirection)
            .xVelocity(79.7783)
            .yVelocity(59.76964)
            ;


    public static OctoConstants localizerConstants = new OctoConstants()
            .portX(0)
            .portY(1)
            .imuScalar(1.03f)
            .forwardPodY(-7.125f)
            .strafePodX(-1.67f)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .forwardEncoderDirection(OctoQuad.EncoderDirection.FORWARD)
            .strafeEncoderDirection(OctoQuad.EncoderDirection.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setLocalizer(new OctoLocalizer(hardwareMap, localizerConstants))
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
