package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subcomponents.AS5600;

// ====== example code ====== //
@TeleOp(name = "test abs encoder")
public class AbsEncoderTest extends LinearOpMode {

    private AS5600 encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        encoder = hardwareMap.get(AS5600.class, RobotConstants.yawTurretEncoderName);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Raw (0-4095)", encoder.getRawAngle());
            telemetry.addData("Raw (0-360)", encoder.getAngleRaw0to360());
            telemetry.addData("Angle (0-360)", encoder.getAngle0to360());
            telemetry.addData("Angle (-180-180)", encoder.getAngle180to180());
            telemetry.update();
        }
    }
}
