package org.firstinspires.ftc.teamcode.testing;

import java.util.ArrayList;
import java.util.Collections;

public class TableInterpolation {
    ArrayList<Double> input;
    ArrayList<Double> output;

    public TableInterpolation(ArrayList<Double> input, ArrayList<Double> output) {
        if (input.size() != output.size())
            throw new IllegalArgumentException("Table interpolation requires the input and output to be the same length.");
        double lastValue = Double.MIN_VALUE;
//        for (int i = 0; i < input.size(); i++) {
//            if (input.get(i) < lastValue) {
//                throw new IllegalArgumentException("Table interpolation requires the input to be sorted.");
//            }
//        }
        this.input = input;
        this.output = output;
    }

    public double interpolate(double d) {
        int idx = -1;
        for (int i = 0; i < input.size(); i++) {
            if (input.get(i) > d) idx = i;
        }
        if (idx == -1 || idx == 0) return 0;
        double a = input.get(idx - 1);
        double b = input.get(idx);
        double p = (d - a) / (b - a);
        double g = output.get(idx - 1);
        double h = output.get(idx);
        return (h - g) * p + g;
    }

    public ArrayList<Double> getInput() { return input; }
    public ArrayList<Double> getOutput() { return output; }
}
