package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.List;

@TeleOp (name = "AprilTagLimelight")
public class AprilTagLimelight extends LinearOpMode {

    private Limelight3A limelight;
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        limelight.start();

        panelsTelemetry.update(telemetry);
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();


                /* todo test this
                try {
                    URL url = new URL("http://limelight.local:5807/results");
                    HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                    conn.setConnectTimeout(100);
                    conn.setReadTimeout(100);
                    conn.setRequestMethod("GET");

                    BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                    StringBuilder response = new StringBuilder();
                    String line;
                    while ((line = in.readLine()) != null) response.append(line);
                    in.close();

                    JSONObject data = new JSONObject(response.toString());
                    JSONObject results = data.getJSONObject("resutls");
                    JSONArray fiducials = results.optJSONArray("Fiducial");

                    if (fiducials == null || fiducials.length() == 0) {
                        telemetry.addLine("NoTags");
                    }
                    else {
                        for (int i = 0; i < fiducials.length(); i++) {
                            JSONObject f = fiducials.getJSONObject(i);
                            int tagID = f.getInt("fID");
                            double tx = f.getDouble("tx");
                            double ty = f.getDouble("ty");
                            telemetry.addData("TagID", tagID);
                            telemetry.addData("tx", tx);
                            telemetry.addData("ty", ty);
                            telemetry.addLine("~~~~~~~~~~~~~~");
                        }
                    }
                }

                catch (Exception e) {
                    telemetry.addData("error", e.getMessage());
                }

                telemetry.update();
                sleep(50);
                */

                // todo try this:
                // panelsTelemetry.addData("id", result.getFiducialResults());

                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                /* todo undelete here after testing
                panelsTelemetry.addData("tx", result.getTx());
                panelsTelemetry.addData("ty", result.getTy());
                panelsTelemetry.addData("txnc", result.getTxNC());
                panelsTelemetry.addData("tync", result.getTyNC());

                //panelsTelemetry.addData("id", result.getBarcodeResults().get(0));


                //todo get apriltag id here
                //panelsTelemetry.addData("id", result.);
                panelsTelemetry.addData("Botpose", botpose.toString());

                 */
            }
            else {
                panelsTelemetry.addData("Limelight", "null data");
            }
            panelsTelemetry.update(telemetry);
        }
        limelight.stop();
    }
}
