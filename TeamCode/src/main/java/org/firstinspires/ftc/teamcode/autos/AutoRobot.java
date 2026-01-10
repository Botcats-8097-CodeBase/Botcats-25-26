package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subcomponents.Intake;
import org.firstinspires.ftc.teamcode.subcomponents.Limelight;
import org.firstinspires.ftc.teamcode.subcomponents.Odometry;
import org.firstinspires.ftc.teamcode.subcomponents.Turret;

public class AutoRobot {
    public Turret turret = new Turret();
    public Limelight limelight = new Limelight();

    int id = 0;

    public AutoRobot() {}

    public void init(HardwareMap hardwareMap, boolean isRed) {
        turret.init(hardwareMap);
        limelight.init(hardwareMap);
    }

    public void update() {
        turret.loop();
    }

    public void faceTarget(Pose pose, boolean isRed) {
        turret.autoFace(pose.getX() - 72, pose.getY() - 72, pose.getHeading() - 90, isRed);
    }

    public void shootSequenceStart(double[] preset) {
        turret.continueShootSequence(preset);
    }

    public boolean isShooting() {
        return turret.currentlyShooting();
    }

    public void shootSequenceUpdate() {

    }
}
