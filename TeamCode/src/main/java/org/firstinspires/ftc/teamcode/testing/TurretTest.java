package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subcomponents.Turret;

import java.util.List;

@Configurable
@TeleOp(name = "test turret")
public class TurretTest extends OpMode {

    List<LynxModule> allHubs;

    Turret turret = new Turret();

    double targetTurretAngle = 0;

    public static double targetSpeed = 1.75;
    public static double targetServoAngle = 0.30;

    double pitchValue = 0;

    ElapsedTime et = new ElapsedTime();

    @Override
    public void init() {
        initRobot();
    }

    @Override
    public void start() {
        super.start();
        et.reset();
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }


        if (gamepad1.a) {
            turret.continueShootSequence(new double[]{targetSpeed, targetServoAngle});
        } else {
            turret.stopShootSequence();
        }

        if (pitchValue > 1) {
            pitchValue = 1;
        } else if (pitchValue < 0) {
            pitchValue = 0;
        }

        if (gamepad1.dpad_up) {
            pitchValue += 0.01;
        } else if (gamepad1.dpad_down) {
            pitchValue -= 0.01;
        } else {
            turret.pitchTurretServo.setPosition(pitchValue);
        }

        telemetry.addData("pitch value", pitchValue);

        telemetry.addData("turret Target Vel", targetSpeed);
        telemetry.addData("turret Current Vel", turret.spinnerMotor1.getVelocity());
        telemetry.addData("turret Current Accel", turret.spinnerMotor1.getCurrentAcceleration());
        telemetry.addData("turret Current Pwr", turret.spinnerMotor1.getPower());
//        telemetry.addData("turret is stopped", turret.spinnerMotor1.isStopped());

        telemetry.addData("targetTurretAngle", targetTurretAngle);
        turret.faceTo(targetTurretAngle);

        telemetry.addData("turret yaw encoder", turret.yawTurretEncoder.getAngle180to180());

        //turret.loop();

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