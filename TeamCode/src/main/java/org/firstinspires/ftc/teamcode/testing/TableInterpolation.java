package org.firstinspires.ftc.teamcode.testing;

import java.util.ArrayList;
import java.util.Collections;

public class TableInterpolation {
    ArrayList<Double> input;
    ArrayList<Double> output;

    public TableInterpolation(ArrayList<Double> input, ArrayList<Double> output) {
        if (input.size() != output.size())
            throw new IllegalArgumentException("Table interpolation requires the input and output to be the same length.");
        double lastValue = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < input.size(); i++) {
            if (input.get(i) < lastValue) {
                throw new IllegalArgumentException("Table interpolation requires the input to be sorted.");
            }
        }
        this.input = input;
        this.output = output;
    }

    public double interpolate(double d) {
        int idx = -1;
        for (int i = 0; i < input.size(); i++) {
            if (input.get(i) < d) idx = i;
        }
        if (idx+1 == input.size() || idx == -1) return 0;
        double a = input.get(idx);
        double b = input.get(idx+1);
        double p = (d - a) / (b - a);
        double g = output.get(idx);
        double h = output.get(idx+1);
        return (h - g) * p + g;
    }

    public ArrayList<Double> getInput() { return input; }
    public ArrayList<Double> getOutput() { return output; }
}
