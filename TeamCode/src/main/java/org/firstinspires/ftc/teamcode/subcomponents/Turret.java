package org.firstinspires.ftc.teamcode.subcomponents;

import static org.firstinspires.ftc.teamcode.utils.TylerMath.normalize180;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.RunToMotor;
import org.firstinspires.ftc.teamcode.utils.VelocityMotor;

public class Turret {
    public RunToMotor yawMotor = new RunToMotor();
    public AS5600 yawTurretEncoder;
    public VelocityMotor spinnerMotor = new VelocityMotor();
    public Servo pitchTurretServo;
    public Servo pusherServo;

    ElapsedTime et = new ElapsedTime();
    ElapsedTime yawTimer = new ElapsedTime();

    double shootStartTimeMs = -1;
    double kickerStartTimeMs = -1;
    double[] targetPreset = {0, 0};

    private double angleDeadbandDeg = 1.0;
    private double derivativeDeadband = 5.0;

    public Turret() {}

    public void init(HardwareMap hardwareMap) {
        yawMotor.init(hardwareMap, RobotConstants.yawTurretMotorName);
        yawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        yawMotor.setTargetPosition(100);
        yawMotor.setPosController(new PIDController(0.015, 0, 0.0005, 0));
        yawMotor.setMaxPower(0.4);

        spinnerMotor.init(hardwareMap, RobotConstants.spinnerMotorName);
        spinnerMotor.setDirection(RobotConstants.spinnerMotorDirection);
        spinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinnerMotor.setMaxPower(1);
        spinnerMotor.setMaxBoundsForTarget(0.01);
        spinnerMotor.setVelController(new PIDFController(1.0, 0, 0, 0.400, 0));

        pitchTurretServo = hardwareMap.get(Servo.class, RobotConstants.pitchTurretServoName);
        pitchTurretServo.setPosition(0.5);

        pusherServo = hardwareMap.get(Servo.class, RobotConstants.pusherServoName);
        pusherServo.setPosition(RobotConstants.pusherStartPos);

        yawTurretEncoder = hardwareMap.get(AS5600.class, RobotConstants.yawTurretEncoderName);


        et.reset();
        yawTimer.reset();
    }

    public void loop() {
        double dt = yawTimer.seconds();
        yawTimer.reset();

        yawMotor.update(yawTurretEncoder.getAngle180to180());

        spinnerMotor.update();

        if (shootStartTimeMs != -1) {
            if (spinnerMotor.isAtTargetVelocity() && kickerStartTimeMs == -1) {
                kickerStartTimeMs = et.milliseconds();
                pusherServo.setPosition(RobotConstants.pusherEndPos);
            }
            if (kickerStartTimeMs != -1) {
                double timeAfterShoot = et.milliseconds() - kickerStartTimeMs;
                if (timeAfterShoot < 500) {

                } else if (timeAfterShoot < 1000) {
                    pusherServo.setPosition(RobotConstants.pusherStartPos);
                } else {
                    shootStartTimeMs = -1;
                    kickerStartTimeMs = -1;
                    stopSpinner();
                }
            }
        }
    }

    public void faceTo(double yaw) {
        yawMotor.setTargetPosition(yaw);
    }

    public void goToPreset(double[] preset) {
        spinnerMotor.setTargetVelocity(preset[0]);
        pitchTurretServo.setPosition(preset[1]);
    }

    public void goToPreset() {
        spinnerMotor.setTargetVelocity(targetPreset[0]);
        pitchTurretServo.setPosition(targetPreset[1]);
    }

    public void continueShootSequence(double[] preset) {
        if (!currentlyShooting()) {
            shootStartTimeMs = et.milliseconds();
            goToPreset(preset);
        }
    }

    public void setShootPreset(double[] preset) {
        this.targetPreset = preset;
    }

    public void stopShootSequence() {
        shootStartTimeMs = -1;
        pusherServo.setPosition(RobotConstants.pusherStartPos);
        stopSpinner();
    }

    public boolean currentlyShooting() {
        return shootStartTimeMs != -1;
    }

    public void stopSpinner() {
        spinnerMotor.stop();
    }

    public double getCurrentFacing() {
        return yawTurretEncoder.getAngle180to180();
    }

    private Double lastAngle0to360 = null;
    private double continuousAngleDeg = 0;
    public double getContinuousTurretAngle() {
        double current = yawTurretEncoder.getAngle180to180();

        if (lastAngle0to360 != null) {
            double delta = current - lastAngle0to360;

            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;

            continuousAngleDeg += delta;
        }

        lastAngle0to360 = current;
        return continuousAngleDeg;
    }

    private double shortestAngleDiff(double target, double current) {
        double diff = target - current;
        diff = normalize180(diff);
        return diff;
    }
}
