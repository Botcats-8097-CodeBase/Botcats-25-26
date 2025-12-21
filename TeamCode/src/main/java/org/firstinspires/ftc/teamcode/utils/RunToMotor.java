package org.firstinspires.ftc.teamcode.utils;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RunToMotor {

    DcMotor motor = null;

    PIDController posController = new PIDController(0.005, 0, 0, 0);

    ElapsedTime deltaTimer = new ElapsedTime();

    double targetPos = 0;
    double maxPower = 0.9;
    double nearDist = 3;

    double currentPower = 0;
    double currentPos = 0;


    public RunToMotor() {}

    public void init(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deltaTimer.reset();
    }

    public void update(double currentPos) {
        this.currentPos = currentPos;
        double dt = deltaTimer.milliseconds();
        deltaTimer.reset();

        currentPower = posController.update(targetPos - currentPos, dt);

        currentPower = clip(currentPower, -maxPower, maxPower);
        if (Math.abs(currentPower) > 0.01)
            motor.setPower(currentPower);
    }

    public void setTargetPosition(double pos) {
        targetPos = pos;
    }

    public double getTargetPosition() {
        return targetPos;
    }

    public double getCurrentPosition() {
        return currentPos;
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

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public PIDController getPosController() {
        return posController;
    }

    public void setPosController(PIDController posController) {
        this.posController = posController;
    }

    public double getNearDist() {
        return nearDist;
    }

    public void setNearDist(double nearDist) {
        this.nearDist = nearDist;
    }

    public double getCurrentPower() {
        return currentPower;
    }

    public void setCurrentPower(double currentPower) {
        this.currentPower = currentPower;
    }
}
