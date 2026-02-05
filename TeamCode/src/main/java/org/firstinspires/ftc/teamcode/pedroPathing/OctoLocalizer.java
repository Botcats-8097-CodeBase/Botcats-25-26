package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OctoLocalizer implements Localizer {

    OctoQuad octoQuad;

    HardwareMap hardwareMap;
    OctoConstants constants;

    OctoQuad.LocalizerDataBlock localizer;

    Pose currPose = new Pose();
    Pose currVel = new Pose();

    double totalHeading = 0;

    public OctoLocalizer(HardwareMap hardwareMap, OctoConstants constants) {
        this.hardwareMap = hardwareMap;
        this.constants = constants;

        octoQuad = hardwareMap.get(OctoQuad.class, constants.hardwareMapName);

        localizer = new OctoQuad.LocalizerDataBlock();

        octoQuad.setLocalizerVelocityIntervalMS(constants.velocitySampleInterval);
        octoQuad.setSingleEncoderDirection(constants.portX, constants.forwardEncoderDirection);
        octoQuad.setSingleEncoderDirection(constants.portY, constants.strafeEncoderDirection);
        octoQuad.setLocalizerPortX(constants.portX);
        octoQuad.setLocalizerPortY(constants.portY);
        octoQuad.setLocalizerCountsPerMM_X(19.894367886f);
        octoQuad.setLocalizerCountsPerMM_Y(19.894367886f);
        octoQuad.setLocalizerTcpOffsetMM_X(constants.strafePodX);
        octoQuad.setLocalizerTcpOffsetMM_Y(constants.forwardPodY);
        octoQuad.setLocalizerImuHeadingScalar(constants.imuScalar);
        octoQuad.setLocalizerVelocityIntervalMS(25);
        octoQuad.setI2cRecoveryMode(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        // Resetting the localizer will apply the parameters configured above.
        // This function will NOT block until calibration of the IMU is complete -
        // for that you need to look at the status returned by getLocalizerStatus()
        octoQuad.resetLocalizerAndCalibrateIMU();
    }

    @Override
    public Pose getPose() {
        return currPose;
    }

    @Override
    public Pose getVelocity() {
        return currVel;
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(currVel.getX(), currVel.getY());
    }

    @Override
    public void setStartPose(Pose setStart) {
        octoQuad.setLocalizerPose((int) setStart.getX(), (int) setStart.getY(), (float) setStart.getHeading());
    }

    @Override
    public void setPose(Pose setPose) {
        octoQuad.setLocalizerPose((int) setPose.getX(), (int) setPose.getY(), (float) setPose.getHeading());
    }

    @Override
    public void update() {
        octoQuad.readLocalizerData(localizer);

        if (localizer.crcOk) {
            double lastHeading = currPose.getHeading();
            currPose = new Pose(localizer.posX_mm, localizer.posY_mm, localizer.heading_rad);
            currVel = new Pose(localizer.velX_mmS, localizer.velY_mmS);

            totalHeading += MathFunctions.getSmallestAngleDifference(currPose.getHeading(), lastHeading) * MathFunctions.getTurnDirection(lastHeading, currPose.getHeading());
        }
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return localizer.posY_mm;
    }

    @Override
    public double getLateralMultiplier() {
        return localizer.posX_mm;
    }

    @Override
    public double getTurningMultiplier() {
        return localizer.heading_rad;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        octoQuad.resetLocalizerAndCalibrateIMU();
    }

    @Override
    public double getIMUHeading() {
        return currPose.getHeading();
    }

    @Override
    public boolean isNAN() {
        return false;
    }
}
