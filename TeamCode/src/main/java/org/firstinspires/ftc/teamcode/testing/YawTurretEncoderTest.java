package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subcomponents.Turret;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.List;

@Configurable
@TeleOp(name = "test yaw turret encoder")
public class YawTurretEncoderTest extends OpMode {

    List<LynxModule> allHubs;
    DcMotor yawTurret;

    private ElapsedTime runtime = new ElapsedTime();

    double pos1 = 0;
    double pos2 = 0;

    @Override
    public void init() {
        yawTurret = hardwareMap.get(DcMotor.class, RobotConstants.yawTurretMotorName);
        yawTurret.setDirection(DcMotorSimple.Direction.FORWARD);
        yawTurret.setTargetPosition(0);
        yawTurret.setPower(0);
    }

    @Override
    public void start() {super.start();}

    @Override
    public void loop() {
        double pos = yawTurret.getCurrentPosition();
        if (gamepad1.aWasPressed()) pos1 = pos;
        if (gamepad1.bWasPressed()) pos2 = pos;

        telemetry.addLine("This function will find the ratio of ticks to degrees. " +
                "Press A for point 1 and B for point 2. " +
                "Move each point 90 degrees from each other");
        telemetry.addData("pos", pos);
        telemetry.addData("pos1", pos1);
        telemetry.addData("pos2", pos2);
        telemetry.addData("ticks per degree", (pos2 - pos1) / 90);
    }
}
