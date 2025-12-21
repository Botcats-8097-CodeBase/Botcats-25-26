package org.firstinspires.ftc.teamcode.subcomponents;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Odometry {
    public double offsetY = -7.125;
    public double offsetX = -1.67;

    public GoBildaPinpointDriver driver;
    public GoBildaPinpointDriver.EncoderDirection initialParDirection;
    public GoBildaPinpointDriver.EncoderDirection initialPerpDirection;

    public Odometry() {}

    public void init(HardwareMap hardwareMap) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = 0.0019702215 * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(offsetY, offsetX, DistanceUnit.INCH);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();
    }

    public void setPose(Pose2D pose) {
        driver.setPosition(pose);
    }

    public Pose2D getPose() {
        return driver.getPosition();
    }

    public void update() {
        driver.update();
    }
}
