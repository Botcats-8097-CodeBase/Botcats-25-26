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
                .hardwareMapName("pinpoint")
                .imuScalar(1.0082761111f)
                .forwardPodY((float) (7.125 * DistanceUnit.mmPerInch))
                .strafePodX((float) (1.67 * DistanceUnit.mmPerInch))
                .portX(0)
                .portY(1)
                .forwardEncoderDirection(OctoQuad.EncoderDirection.REVERSE)
                .strafeEncoderDirection(OctoQuad.EncoderDirection.REVERSE);

        localizer = new OctoLocalizer(hardwareMap, localizerConstants);
    }

    public void setPose(Pose2D pose) {
        localizer.setPose(new Pose(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS)));
    }

    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.INCH, localizer.getPose().getX(), localizer.getPose().getY(), AngleUnit.RADIANS, localizer.getPose().getHeading());
    }

    public void reset() {
        try {
            localizer.resetIMU();
        } catch(InterruptedException e) {

        }
    }

    public void update() {
        localizer.update();
    }
}