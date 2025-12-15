package org.firstinspires.ftc.teamcode.utils;

public class NoiseFilter {
    double lastPos;
    boolean isDataless = true;
    double weight;
    double errorBounds = -1;

    public NoiseFilter(double weight, double errorBounds) {
        this.weight = weight;
        this.errorBounds = errorBounds;
    }

    public void update(double pos) {
        if (Math.abs(lastPos - pos) < errorBounds) {
            lastPos = (1 - weight) * lastPos + weight * pos;
            isDataless = false;
        }
    }

    public double getPos() {
        return lastPos;
    }

    public boolean isDataless() {
        return isDataless;
    }

    public void reset() {
        isDataless = true;
        lastPos = 0;
    }
}