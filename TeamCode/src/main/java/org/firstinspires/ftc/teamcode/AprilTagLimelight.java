package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp (name = "AprilTagLimelight")
@Disabled
public class AprilTagLimelight extends LinearOpMode {

    private Limelight3A limelight;

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

                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("tync", result.getTyNC());


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
