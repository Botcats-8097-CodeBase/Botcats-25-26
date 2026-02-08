package org.firstinspires.ftc.teamcode.subcomponents;

import static com.qualcomm.robotcore.util.Range.clip;

import static org.firstinspires.ftc.teamcode.utils.TylerMath.normalize180;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;


@Configurable
public class Limelight {
    public enum MosaicPattern {
        PPG(0),
        PGP(1),
        GPP(2);

        private final int value;

        MosaicPattern(int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }

        public static MosaicPattern getPattern(int index) {
            for (MosaicPattern value : MosaicPattern.values()) {
                if (index == value.value) {
                    return value;
                }
            }
            return null;
        }
    }

    private Limelight3A limelight;
    double filteredTx = 0;

    public Limelight() {}

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }


    public Double limeAutoFacing(double currentFacing, int id) {
        double tx = -limeScanPosX(id);
        if (!Double.isNaN(tx)) {
            double alpha = 0.3;
            filteredTx = filteredTx + alpha * (tx - filteredTx);

            double kVision = 1.0;

            double newAngle = normalize180(currentFacing + kVision * filteredTx);
            newAngle = clip(newAngle, RobotConstants.yawTurretMinAngle, RobotConstants.yawTurretMaxAngle);


            return newAngle;
        }
        else {
            return null;
        }
    }

    // Outputs in meters and degrees
    public Pose3D limePosFace() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult f : fiducialResults) {
                if (f.getFiducialId() == 20 || f.getFiducialId() == 24) {
                    return f.getRobotPoseFieldSpace();
                }
            }
        }
        return null;
    }

    public static double[] offsetTurret(double yaw, double distance) {
        double radian = Math.toRadians(yaw);
        return new double[]{Math.cos(radian) * distance, Math.sin(radian) * distance};
    }

    public MosaicPattern getMosaicPattern() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() < 24 && fr.getFiducialId() > 20) {
                    return MosaicPattern.getPattern(fr.getFiducialId() - 21);
                }
            }
        }
        return null;
    }

    public boolean limeNullCheck() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                    return true;
                }
            }
        }
        return false;
    }

    public double limeScanPosX(int tagID) {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult f : fiducialResults) {
                if (f.getFiducialId() == tagID) {
                    return f.getTargetXDegrees();
                }
            }
        }
        return Double.NaN;
    }

    public double limeScanPosY(int tagID) {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult f : fiducialResults) {
                if (f.getFiducialId() == tagID) {
                    return f.getTargetYDegrees();
                }
            }
        }
        return Double.NaN;
    }
}