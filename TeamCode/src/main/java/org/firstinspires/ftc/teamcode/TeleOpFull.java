package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.actions.ActionManager;
import org.firstinspires.ftc.teamcode.subcomponents.Limelight;
import org.firstinspires.ftc.teamcode.subcomponents.Odometry;
import org.firstinspires.ftc.teamcode.subcomponents.Turret;
import org.firstinspires.ftc.teamcode.utils.BasicRobot;
import org.firstinspires.ftc.teamcode.utils.TylerMath;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@TeleOp(name = "!teleopfull")
public class TeleOpFull extends OpMode {

    List<LynxModule> allHubs;

    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    BasicRobot robot = new BasicRobot();
    Turret turret = new Turret();
    Limelight limelight = new Limelight();
    Odometry odo = new Odometry();

    boolean isRed = false;
    String[] color = {"blue", "red"};
    boolean useAbsToReset = false;

    boolean isClose = true;
    boolean isBlackBoardPos = true;

    boolean isConstantPreset = false;

    double yawOffset = -90;
    double turretOffset = 0;

    int id;

    double targetTurretAngle = 0;
    boolean isAutoAiming = true;

    ElapsedTime et = new ElapsedTime();

    Pose2D initPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    double lastTimeSeenLimelight = 0;

    @Override
    public void loop() {
        float dt = (float) et.milliseconds();
        pTelemetry.addData("dt", dt);
        et.reset();

        for (LynxModule hub : allHubs) hub.clearBulkCache();

        odo.update();
        Pose2D robotPos = odo.getPose();

        // reset IMU
        if (gamepad1.yWasPressed()) {
            odo.setPose(new Pose2D(DistanceUnit.INCH, robotPos.getX(DistanceUnit.INCH), robotPos.getY(DistanceUnit.INCH), AngleUnit.DEGREES, 180));
        }

        double yaw = TylerMath.wrap(robotPos.getHeading(AngleUnit.DEGREES), 0, 360);
        pTelemetry.addData("Robot Yaw (imu)", yaw);

        // pressurising the right trigger slows down the drive train
        double coefficient = 1;
        if (gamepad1.right_trigger < 0.5) pTelemetry.addData("Speed Mode", "on");
        else
        {
            pTelemetry.addData("Speed Mode", "off");
            coefficient = 0.35;
        }

        robot.drivePower(
                new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y)
                        .rotateVector(-Math.toRadians(yaw-yawOffset))
                , gamepad1.right_stick_x, coefficient);

        if (gamepad1.right_bumper || gamepad2.right_bumper) turret.triggerIntake();
        else if (gamepad1.left_bumper || gamepad2.left_bumper) turret.reverseIntake();
        else turret.stopIntake();

        boolean dpadRight = gamepad1.dpad_right;
        if (dpadRight) {
            turret.manualOverride();
        }

        pTelemetry.addData("is auto aiming", isAutoAiming);

        if (gamepad1.dpadLeftWasPressed()) isConstantPreset = !isConstantPreset;
        if (gamepad1.backWasPressed()) isAutoAiming = !isAutoAiming;
        if (gamepad2.backWasPressed()) {
            turretOffset = 0;
            turret.resetYawPos();
        }

        if (gamepad1.a) turret.continueShootSequence();
        else turret.stopShootSequence();

        turret.spinUp(isConstantPreset);
        turret.loop();

        if (!isAutoAiming) {
            if (gamepad1.dpad_down) targetTurretAngle -= 2;
            if (gamepad1.dpad_up) targetTurretAngle += 2;

            pTelemetry.addData("targetTurretAngle", targetTurretAngle);

            turret.faceTo(targetTurretAngle);

            if (gamepad2.yWasPressed()) {
                targetTurretAngle = 0;
                isAutoAiming = true;
            }

        } else {
            if (gamepad1.dpad_down) turretOffset += 2;
            if (gamepad1.dpad_up) turretOffset -= 2;

            double turretLimelightDistance = 5.5;
            double turretBodyDistance = 1;
            Pose3D camPose = limelight.limePosFace();

            if (camPose == null) {
                if ((getRuntime() - lastTimeSeenLimelight) > 0.8) {
                    turret.updatePose(new double[]{robotPos.getX(DistanceUnit.INCH), robotPos.getY(DistanceUnit.INCH), yaw});
                    double facingTarget = turret.autoFace() + turretOffset;
                    turret.faceTo(facingTarget);


                    pTelemetry.addData("facingTarget", facingTarget);
                    pTelemetry.addData("distError", turret.distError);
                }
            } else {
                camPose.getPosition().x *= 39.37;
                camPose.getPosition().y *= 39.37;
                double[] limelightOffset = Limelight.offsetTurret(yaw - turret.getCurrentFacing() - 3, turretLimelightDistance);
                camPose.getPosition().x += limelightOffset[0];
                camPose.getPosition().y += limelightOffset[1];
                pTelemetry.addData("lime offset x", limelightOffset[0]);
                pTelemetry.addData("lime offset y", limelightOffset[1]);
                limelightOffset = Limelight.offsetTurret(yaw - 170, turretBodyDistance);
                camPose.getPosition().x += limelightOffset[0];
                camPose.getPosition().y += limelightOffset[1];

                turret.updatePose(new double[]{camPose.getPosition().x, camPose.getPosition().y, yaw});
                double limePosFace = turret.autoFace() + turretOffset;
                turret.faceTo(limePosFace);

                lastTimeSeenLimelight = getRuntime();
                pTelemetry.addData("limePosFace", limePosFace);
                pTelemetry.addData("lime pos x", camPose.getPosition().x);
                pTelemetry.addData("lime pos y", camPose.getPosition().y);

                pTelemetry.addData("lime offset x", limelightOffset[0]);
                pTelemetry.addData("lime offset y", limelightOffset[1]);
            }
        }

        pTelemetry.addData("x", robotPos.getX(DistanceUnit.INCH));
        pTelemetry.addData("y", robotPos.getY(DistanceUnit.INCH));
        pTelemetry.addData("pinpoint yaw", robotPos.getHeading(AngleUnit.DEGREES));



        pTelemetry.update();

        ActionManager.update();

    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot.init(hardwareMap);
        turret.init(hardwareMap);
        limelight.init(hardwareMap);
        odo.init(hardwareMap);

        turret.pTelemetry = pTelemetry;
    }

    @Override
    public void init_loop() {
        int colorNum = isRed ? 1 : 0;
        id = isRed ? 24 : 20;
        pTelemetry.addData("team", color[colorNum]);
        pTelemetry.addData("is close", isClose);
        pTelemetry.addData("is black board pos", isBlackBoardPos);
        pTelemetry.addData("use abs encoder to reset", useAbsToReset);
        pTelemetry.addData("x", blackboard.get("x"));
        pTelemetry.addData("y", blackboard.get("y"));
        pTelemetry.addData("heading", blackboard.get("heading"));
        pTelemetry.addData("angle", turret.yawTurretEncoder.getAngle180to180());

        pTelemetry.update();

        if (gamepad1.aWasPressed()) isRed = !isRed;
        if (gamepad1.bWasPressed()) isClose = !isClose;
        if (gamepad1.xWasPressed()) isBlackBoardPos = !isBlackBoardPos;
        if (gamepad1.backWasPressed()) useAbsToReset = !useAbsToReset;

        // reset turret 0
        if (gamepad1.yWasPressed()) {
            turret.resetYawPos();
        }
    }

    @Override
    public void start() {
        super.start();
        et.reset();

        if (isBlackBoardPos && blackboard.get("x") != null && blackboard.get("y") != null && blackboard.get("heading") != null && blackboard.get("yawPos") != null) {
            initPose = new Pose2D(
                    DistanceUnit.INCH,
                    (double) blackboard.get("x"),
                    (double) blackboard.get("y"),
                    AngleUnit.DEGREES,
                    TylerMath.wrap(((double) blackboard.get("heading")), -180, 180));
            turret.setYawPos((double) blackboard.get("yawPos"));
        } else {
            if (!isRed) {
                if (isClose) initPose = new Pose2D(DistanceUnit.INCH, 63.5, -8.5, AngleUnit.DEGREES, 180); // 65.315 -8.38 180 270
                else initPose = new Pose2D(DistanceUnit.INCH, -63.5, -8.5, AngleUnit.DEGREES, 180);

            } else {
                if (isClose) initPose = new Pose2D(DistanceUnit.INCH, 63.5, 8.5, AngleUnit.DEGREES, 180);
                else initPose = new Pose2D(DistanceUnit.INCH, -63.5, 8.5, AngleUnit.DEGREES, 180);
            }
        }

        if (!isRed) {
            yawOffset = -90;
        } else {
            yawOffset = 90;
        }

        if (useAbsToReset) {
            turret.useAbsToReset();
        }

        odo.setPose(initPose);
        turret.setGoalColor(isRed);
    }
}