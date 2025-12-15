package org.firstinspires.ftc.teamcode.utils;

import static com.pedropathing.math.MathFunctions.clamp;

public class PIDController {

    double lastError = 0;

    double integralSum = 0;

    double Kp, Ki, Kd, maxIntegral = 0;

    public PIDController(double Kp, double Ki, double Kd, double maxIntegral) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
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

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;

        return out;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
    }
}