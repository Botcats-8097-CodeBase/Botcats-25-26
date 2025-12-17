package org.firstinspires.ftc.teamcode.subcomponents;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.teamcode.RobotConstants;

@I2cDeviceType
@DeviceProperties(
        name = "AS5600 Angle Sensor",
        xmlTag = "AS5600"
)
public class AS5600 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final I2cAddr AS5600_ADDR = I2cAddr.create7bit(0x36);

    private static final int ANGLE_HIGH_REG = 0x0E;
    private static final int ANGLE_LOW_REG  = 0x0F;

    public AS5600(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(AS5600_ADDR);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "AS5600 Absolute Encoder";
    }

    // --- public API ---

    public double directionSign = -1.0;
    public double offsetDeg = 0;

    public int getRawAngle() {
        byte[] data = deviceClient.read(ANGLE_HIGH_REG, 2);
        int high = data[0] & 0xFF;
        int low = data[1] & 0xFF;
        return ((high << 8) | low) & 0x0FFF;
    }

    public double getAngleRaw0to360() {
        return getRawAngle() * (360.0 / 4096.0);
    }

    public double getAngle0to360() {
        double angle = getAngleRaw0to360();
        angle -= RobotConstants.yawTurretStartEncoder;
        angle *= directionSign;

        return angle;
    }

    public double getAngle180to180() {
        double a = getAngle0to360();
        if (a > 180.0) a -= 360.0;
        return a;
    }

    public void zeroHere() {
        double angle = getAngleRaw0to360() * directionSign;
        RobotConstants.yawTurretStartEncoder = angle;
    }
}
