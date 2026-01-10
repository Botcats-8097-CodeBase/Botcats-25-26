package org.firstinspires.ftc.teamcode.subcomponents;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class CustomColorSensor {
    public ColorSensor lowColorSensor;

    float color1 = 215;
    float color2 = 157;
    float value = 0.5f;
    float colorBounds = 20;

    DcMotor intakeMotor;

    public CustomColorSensor() {}

    public void init(HardwareMap hardwareMap, String name) {
        lowColorSensor = hardwareMap.get(ColorSensor.class, name);
        lowColorSensor.enableLed(true);
    }

    public boolean checkColor() {
        float[] hsv = new float[3];
        Color.RGBToHSV(lowColorSensor.red(), lowColorSensor.green(), lowColorSensor.blue(), hsv);

        return (Math.abs(hsv[0] - color1) < colorBounds || Math.abs(hsv[0] - color2) < colorBounds) && hsv[2] > value;
    }
}