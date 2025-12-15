package org.firstinspires.ftc.teamcode.utils;

public class TylerMath {
    public static double normalize180(double theta) {
        return wrap(theta, -180, 180);
    }

    public static double normalize360(double theta) {
        return wrap(theta, 0, 360);
    }

    // An expanded modulo that includes a range and negative numbers.
    public static double wrap(double a, double min, double max) {
        double d = max - min;
        return (((((a - min) % d) + d) % d) + min);
    }
}
