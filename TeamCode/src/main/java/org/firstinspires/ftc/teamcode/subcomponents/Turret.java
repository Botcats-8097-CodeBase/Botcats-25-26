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
    public VelocityMotor spinnerMotor1 = new VelocityMotor();
    public DcMotor spinnerMotor2;
    public Servo pitchTurretServo;
    public Servo pusherServo;

    ElapsedTime et = new ElapsedTime();
    ElapsedTime yawTimer = new ElapsedTime();

    double shootStartTimeMs = -1;
    double kickerStartTimeMs = -1;
    double[] targetPreset = {0, 0};

    public Turret() {}

    public void init(HardwareMap hardwareMap) {
        yawMotor.init(hardwareMap, RobotConstants.yawTurretMotorName);
        yawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        yawMotor.setTargetPosition(100);
        yawMotor.setPosController(new PIDController(0.015, 0, 0.0005, 0));
        yawMotor.setMaxPower(0.4);

        spinnerMotor1.init(hardwareMap, RobotConstants.spinnerMotor1Name);
        spinnerMotor1.setDirection(RobotConstants.spinnerMotor1Direction);
        spinnerMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinnerMotor1.setMaxPower(1);
        spinnerMotor1.setMaxBoundsForTarget(0.01);
        spinnerMotor1.setVelController(new PIDFController(1.2, 0.002, 0, 0.420, 100));

        spinnerMotor2 = hardwareMap.get(DcMotor.class, RobotConstants.spinnerMotor2Name);
        spinnerMotor2.setDirection(RobotConstants.spinnerMotor2Direction);
        spinnerMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinnerMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerMotor2.setPower(0);

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

        spinnerMotor1.update();
        spinnerMotor2.setPower(spinnerMotor1.getPower());

        if (shootStartTimeMs != -1) {
            if (spinnerMotor1.isAtTargetVelocity() && kickerStartTimeMs == -1) {
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
        spinnerMotor1.setTargetVelocity(preset[0]);
        pitchTurretServo.setPosition(preset[1]);
    }

    public void goToPreset() {
        spinnerMotor1.setTargetVelocity(targetPreset[0]);
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
        spinnerMotor1.stop();
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
