package org.firstinspires.ftc.teamcode.utils;

import static com.pedropathing.math.MathFunctions.clamp;

public class PIDFController {

    double lastError = 0;

    double integralSum = 0;

    double Kp, Ki, Kd, Kv, targetVelocity, maxIntegral = 0;

    double lastOut = 0;

    public PIDFController(double Kp, double Ki, double Kd, double Kv, double maxIntegral) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kv = Kv;
        this.maxIntegral = maxIntegral;
    }


    public double update(double error, double dt) {

        double derivative;
        // rate of change of the error
        if (dt <= 0) derivative = (error - lastError) / dt;
        else derivative = 0;

        // sum of all error over time
        if ((error * dt) != 0.0f) integralSum += (error * dt);
        if (maxIntegral != 0) integralSum = clamp(integralSum, -maxIntegral, maxIntegral);

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kv * targetVelocity);

        lastError = error;

        lastOut = out;

        return out;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
    }

    public double getOut() {
        return lastOut;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }


}