package org.firstinspires.ftc.teamcode.utils;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class VelocityMotor {

    DcMotor motor = null;
    PIDFController velController = new PIDFController(1.2f, 0, 0, 0.4, 0);

    ElapsedTime deltaTimer = new ElapsedTime();

    double targetVelocity = 0;

    // errorBounds before: 5
    public NoiseFilter velocityFilter = new NoiseFilter(0.3, 25);
    NoiseFilter accelerationFilter = new NoiseFilter(0.3, 5);

    boolean isStopped = true;
    double currentPower = 0;
    double lastPos = 0;
    double maxPower = 0.9;
    // before: 0.0001
    double maxAccelForTarget = 0.0001;

    // before: 0.1
    double maxBoundsForTarget = 0.2;

    public VelocityMotor() {}

    public void init(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deltaTimer.reset();
    }

    public void setTargetVelocity(double vel) {
        isStopped = false;
        targetVelocity = vel;
        velController.setTargetVelocity(vel);
    }

    public boolean isAtTargetVelocity() {
        if (velocityFilter.isDataless()) return false;// || accelerationFilter.isDataless()) return false;
        return Math.abs(targetVelocity - velocityFilter.getPos()) < maxBoundsForTarget;// && Math.abs(accelerationFilter.getPos()) < maxAccelForTarget ;
    }

    public void update() {
        double dt = deltaTimer.milliseconds();
        deltaTimer.reset();

        double lastVelocity = velocityFilter.getPos();
        velocityFilter.update((motor.getCurrentPosition()-lastPos) / dt);
        lastPos = motor.getCurrentPosition();

        accelerationFilter.update((velocityFilter.getPos() - lastVelocity) / dt);
        velController.update(targetVelocity - velocityFilter.getPos(), dt);

        if (!isStopped && !velocityFilter.isDataless() && !accelerationFilter.isDataless()) {
            currentPower = velController.getOut();

            currentPower = clip(currentPower, -maxPower, maxPower);
            if (Math.abs(currentPower) > 0.01)
                motor.setPower(currentPower);
        } else {
            currentPower = 0;
            motor.setPower(0);
        }
    }

    public void stop() {
        isStopped = true;
        velocityFilter.reset();
        accelerationFilter.reset();
        velController.reset();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentAcceleration() {
        return accelerationFilter.getPos();
    }

    public void setVelController(PIDFController controller) {
        velController = controller;
    }

    public double getVelocity() {
        return velocityFilter.getPos();
    }

    public double getPower() {
        return currentPower;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public double getMaxBoundsForTarget() {
        return maxBoundsForTarget;
    }

    public void setMaxBoundsForTarget(double maxBoundsForTarget) {
        this.maxBoundsForTarget = maxBoundsForTarget;
    }

    public double getMaxAccelForTarget() {
        return maxAccelForTarget;
    }

    public void setMaxAccelForTarget(double maxAccelForTarget) {
        this.maxAccelForTarget = maxAccelForTarget;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
}