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

    public int limeScanID() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() < 24 && fr.getFiducialId() > 20) {
                    return fr.getFiducialId();
                }
            }
        }
        return 0;
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