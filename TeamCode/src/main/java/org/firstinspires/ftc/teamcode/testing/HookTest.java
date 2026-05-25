package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subcomponents.Turret;

import java.util.List;

@Configurable
@TeleOp(name = "test hook")
public class HookTest extends OpMode {

    List<LynxModule> allHubs;
    public Servo hookServo;


    private ElapsedTime runtime = new ElapsedTime();

    double serverPos = 0;

    @Override
    public void init() {
        hookServo = hardwareMap.get(Servo.class, "hook");
    }

    @Override
    public void start() {super.start();}

    @Override
    public void loop() {
        hookServo.setPosition(serverPos);
        telemetry.addData("servo pos", serverPos);

        if (gamepad1.a) {
            serverPos += 0.001;
        }
        if (gamepad1.b) {
            serverPos -= 0.001;
        }
        if (serverPos > 1) {
            serverPos = 1;
        }
        if (serverPos < 0) {
            serverPos = 0;
        }
    }
}
