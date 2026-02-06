package org.firstinspires.ftc.teamcode.subcomponents;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.OctoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.OctoLocalizer;

public class Odometry {
    OctoLocalizer localizer;
    public Odometry() {}

    public void init(HardwareMap hardwareMap) {
        OctoConstants localizerConstants = new OctoConstants()
                .portX(0)
                .portY(1)
                .imuScalar(1.03f)
                .forwardPodY(-7.125f)
                .strafePodX(-1.67f)
                .portX(1)
                .portY(0)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .forwardEncoderDirection(OctoQuad.EncoderDirection.REVERSE)
                .strafeEncoderDirection(OctoQuad.EncoderDirection.REVERSE);

        localizer = new OctoLocalizer(hardwareMap, localizerConstants);
    }

    public void setPose(Pose2D pose) {
        localizer.setPose(new Pose(pose.getX(DistanceUnit.MM), pose.getY(DistanceUnit.MM), pose.getHeading(AngleUnit.RADIANS)));
    }

    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.MM, localizer.getPose().getX(), localizer.getPose().getY(), AngleUnit.RADIANS, localizer.getPose().getHeading());
    }

    public void update() {
        localizer.update();
    }
}
