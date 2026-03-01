package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subcomponents.Turret;

import java.util.List;

@TeleOp(name = "test dumb turret")
public class TurretDumbTest extends OpMode {

    List<LynxModule> allHubs;

    public Servo pitchTurretServo;

    double targetTurretAngle = 0;

    public static double targetSpeed = 1.75;
    public static double targetServoAngle = 0.30;

    @Override
    public void init() {
        initRobot();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        if (gamepad1.dpad_down) targetSpeed -= 0.01;
        if (gamepad1.dpad_up) targetSpeed += 0.01;

        if (gamepad1.dpad_left) targetTurretAngle -= 0.001;
        if (gamepad1.dpad_right) targetTurretAngle += 0.001;

        telemetry.addData("turret preset angle", targetTurretAngle);

        pitchTurretServo.setPosition(targetTurretAngle);

        telemetry.update();
    }

    void initRobot() {
        allHubs = hardwareMap.getAll(LynxModule.class);


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        pitchTurretServo = hardwareMap.get(Servo.class, RobotConstants.pitchTurretServoName);
        pitchTurretServo.setPosition(0.5);

        //telemetry.addData("turret Enc", turret.yawTurretMotor.getCurrentPosition());
    }
}