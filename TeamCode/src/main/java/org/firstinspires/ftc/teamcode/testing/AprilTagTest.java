package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp (name = "test apriltag")
public class AprilTagTest extends LinearOpMode {

    private Limelight3A limelight;
    private int autoNum;


    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        limelight.start();

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();

                // todo try this for panels (low priority):
                // panelsTelemetry.addData("id", result.getFiducialResults());

                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() <= 23 && fr.getFiducialId() >= 21) {
                        //telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        autoNum = fr.getFiducialId();

                        break;
                        //telemetry.addData("Fiducial", "ID: %d, Family: %s, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetYDegrees());
                    }
                }

                telemetry.addData("ID to be used:", autoNum);

                /*
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("tync", result.getTyNC());
                */
                telemetry.addData("Botpose", botpose.toString());


            }
            else {
                telemetry.addData("Limelight", "null data");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}