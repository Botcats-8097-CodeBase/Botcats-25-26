package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subcomponents.Turret;

import java.util.List;

@TeleOp(name = "test turret")
public class TurretTest extends OpMode {

    List<LynxModule> allHubs;

    Turret turret = new Turret();

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

        turret.setShootPreset(new double[]{targetSpeed, targetServoAngle});

        if (gamepad1.a) turret.continueShootSequence();
        else turret.stopShootSequence();

        if (gamepad1.dpad_down) targetSpeed -= 0.001;
        if (gamepad1.dpad_up) targetSpeed += 0.001;

        if (gamepad1.dpad_left) targetServoAngle -= 0.001;
        if (gamepad1.dpad_right) targetServoAngle += 0.001;

        if (gamepad1.right_bumper || gamepad2.right_bumper) turret.triggerIntake();
        else if (gamepad1.left_bumper || gamepad2.left_bumper) turret.reverseIntake();
        else turret.stopIntake();

        telemetry.addData("turret preset speed", targetSpeed);
        telemetry.addData("turret preset angle", targetServoAngle);

        telemetry.addData("turret Target Vel", targetSpeed);
        telemetry.addData("turret Current Vel", turret.spinnerMotor1.getVelocity());
        telemetry.addData("turret Current Accel", turret.spinnerMotor1.getCurrentAcceleration());
        telemetry.addData("turret Current Pwr", turret.spinnerMotor1.getPower());
//        telemetry.addData("turret is stopped", turret.spinnerMotor1.isStopped());

        turret.faceTo(targetTurretAngle);

        telemetry.addData("turret yaw encoder", turret.yawTurretEncoder.getAngle180to180());

        turret.loop();

        telemetry.update();
    }

    void initRobot() {
        allHubs = hardwareMap.getAll(LynxModule.class);


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        turret.init(hardwareMap);

        //telemetry.addData("turret Enc", turret.yawTurretMotor.getCurrentPosition());
    }
}