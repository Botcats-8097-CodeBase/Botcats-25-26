package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subcomponents.Limelight;
import org.firstinspires.ftc.teamcode.subcomponents.Turret;

public class AutoRobot {
    public Turret turret = new Turret();
    public Limelight limelight = new Limelight();

    int id = 0;

    public AutoRobot() {}

    public void init(HardwareMap hardwareMap) {
        turret.init(hardwareMap);
        limelight.init(hardwareMap);
    }

    public void update() {
        turret.loop();
    }
}
