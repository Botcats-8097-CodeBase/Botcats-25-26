package org.firstinspires.ftc.teamcode.pedroPathing;

public class PedroConversion {
    public static double[] pedroToOdo(double[] pos) {
        return new double[]{-pos[1] + 72, pos[0] - 72, Math.toDegrees(pos[2]) - 90};
    }

    public static double[] odoToPedro(double[] pos) {
        return new double[]{pos[1] + 72, -(pos[0] - 72), Math.toRadians(pos[2] + 90)};
    }

}
