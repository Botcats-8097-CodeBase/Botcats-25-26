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

    public double getTargetVelocity() {
        return targetVelocity;
    }

    double targetVelocity = 0;

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    double maxVelocity = 50;
    double maxAcceleration = 10;

    double currentVelocity = 0;

    public double getCurrentAcceleration() {
        return currentAcceleration;
    }

    public void setCurrentAcceleration(double currentAcceleration) {
        this.currentAcceleration = currentAcceleration;
    }

    double currentAcceleration = 0;
    ArrayList<Double> velocities = new ArrayList<>();
    ArrayList<Double> accelerations = new ArrayList<>();
    double currentPower = 0;
    double lastPos = 0;
    double maxPower = 0.9;
    double maxAccelForTarget = 0.0001;

    public boolean isStopped() {
        return isStopped;
    }

    public void setStopped(boolean stopped) {
        isStopped = stopped;
    }

    boolean isStopped = false;
    boolean hasBeenStopped = false;
    double maxBoundsForTarget = 0.1;

    public VelocityMotor() {}

    public void init(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deltaTimer.reset();
    }

    public void setVelController(PIDFController controller) {
        velController = controller;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public double getPower() {
        return currentPower;
    }

    public void setTargetVelocity(double vel) {
        targetVelocity = vel;
        velController.setTargetVelocity(vel);
        isStopped = false;
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(targetVelocity - currentVelocity) < maxBoundsForTarget && Math.abs(currentAcceleration) < maxAccelForTarget ;
    }

    public void update() {
        double dt = deltaTimer.milliseconds();
        deltaTimer.reset();

        double lastVelocity = currentVelocity;
        double rawCurrentVelocity = (lastPos-motor.getCurrentPosition()) / dt;
        velocities.add(rawCurrentVelocity);
        if (velocities.size() > 30) velocities.remove(0);
        currentVelocity = 0;
        for (int i = 0; i < velocities.size(); i++) {
            currentVelocity += velocities.get(i);
        }
        currentVelocity /= velocities.size();
        currentVelocity = clip(currentVelocity, -maxVelocity, maxVelocity);

        double rawAcceleration = (currentVelocity - lastVelocity) / dt;
        accelerations.add(rawAcceleration);
        if (accelerations.size() > 30) accelerations.remove(0);
        currentAcceleration = 0;
        for (int i = 0; i < accelerations.size(); i++) {
            currentAcceleration += accelerations.get(i);
        }
        currentAcceleration /= accelerations.size();
        currentAcceleration = clip(currentAcceleration, -maxAcceleration, maxAcceleration);

        lastPos = motor.getCurrentPosition();

        if (!isStopped) {
            if (hasBeenStopped) {
                velController.reset();
                hasBeenStopped = false;
            }

            currentPower = velController.update(targetVelocity - currentVelocity, dt);

            // THIS DOES NOT WORK NORMALLY. THE MIN POWER CAPS AT 0 NOT -maxPower
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
        hasBeenStopped = true;
        velController.reset();
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