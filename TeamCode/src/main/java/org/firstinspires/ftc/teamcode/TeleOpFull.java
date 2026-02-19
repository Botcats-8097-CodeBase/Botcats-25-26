package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import android.graphics.Color;

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
import org.firstinspires.ftc.teamcode.subcomponents.CustomColorSensor;
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

    boolean isClose = true;
    boolean isBlackBoardPos = true;

    boolean isConstantPreset = false;

    double yawOffset = 90;
    double imuOffset = 0;

    private boolean dpadRightPrev = false;

    int id;

    public IMU imu;
    double targetTurretAngle = 0;
    boolean isAutoAiming = true;

    ElapsedTime et = new ElapsedTime();

    Pose2D initPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    double lastTimeSeenLimelight = 0;

    @Override
    public void loop() {
        float dt = (float) et.milliseconds();
        pTelemetry.addData("dt", dt);
        et.reset();

        for (LynxModule hub : allHubs) hub.clearBulkCache();

        // reset IMU
        if (gamepad1.yWasPressed()) {
            imu.resetYaw();
            imu.initialize(parameters);
            imuOffset = 0;
        }

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = TylerMath.wrap(robotOrientation.getYaw(AngleUnit.DEGREES) + imuOffset, 0, 360);
        pTelemetry.addData("Robot Yaw (imu)", yaw);

        // pressurising the right trigger slows down the drive train
        double coefficient = 0.35;
        if(gamepad1.right_trigger < 0.5) pTelemetry.addData("Speed Mode", "off");
        else
        {
            pTelemetry.addData("Speed Mode", "on");
            coefficient = 1;
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
        if (!dpadRight && dpadRightPrev) {
            turret.stopManualOverride();
        }
        dpadRightPrev = dpadRight;


        if (gamepad1.dpad_up) turret.clutchServo.setPosition(RobotConstants.clutchStartPos);
        if (gamepad1.dpad_down) turret.clutchServo.setPosition(RobotConstants.clutchEndPos);

        if (gamepad1.backWasPressed()) isAutoAiming = !isAutoAiming;

        pTelemetry.addData("is auto aiming", isAutoAiming);

        if (gamepad1.dpadLeftWasPressed()) isConstantPreset = !isConstantPreset;

        odo.update();
        Pose2D robotPos = odo.getPose();

        if (gamepad1.a) turret.continueShootSequence();
        else {
            if (isConstantPreset) turret.spinUp();
            turret.isShooting = false;
        }

        pTelemetry.addData("turret Vel", turret.spinnerMotor1.getVelocity());
        pTelemetry.addData("turret Pwr", turret.spinnerMotor1.getPower());
        pTelemetry.addData("turret Target Vel", turret.getCurrentTargets()[0]);
        pTelemetry.addData("turret Target Pitch", turret.getCurrentTargets()[1]);

        if (!isAutoAiming) {
            if (gamepad1.dpad_down) targetTurretAngle -= 1;
            if (gamepad1.dpad_up) targetTurretAngle += 1;

            pTelemetry.addData("targetTurretAngle", targetTurretAngle);

            turret.faceTo(targetTurretAngle);

            if (gamepad2.yWasPressed()) {
                targetTurretAngle = 0;
                isAutoAiming = true;
            }

        } else {
            if (gamepad1.dpad_down) RobotConstants.yawTurretStartAngle += 1;
            if (gamepad1.dpad_up) RobotConstants.yawTurretStartAngle -= 1;

            double limePosFace = turret.autoFace();

            double turretLimelightDistance = 5.5;
            Pose3D camPose = limelight.limePosFace();


            if (camPose == null) {
                if ((getRuntime() - lastTimeSeenLimelight) > 0.8) {
                    turret.updatePose(new double[]{robotPos.getX(DistanceUnit.INCH), robotPos.getY(DistanceUnit.INCH), yaw});
                    double facingTarget = turret.autoFace();
                    pTelemetry.addData("facingTarget", facingTarget);
                    turret.faceTo(facingTarget);

                    pTelemetry.addData("distError", turret.distError);
                }
            } else {
                camPose.getPosition().x *= 39.37;
                camPose.getPosition().y *= 39.37;
                double[] limelightOffset = Limelight.offsetTurret(-yaw + turret.getCurrentFacing(), turretLimelightDistance);
                camPose.getPosition().x += limelightOffset[0];
                camPose.getPosition().y += limelightOffset[1];
                turret.updatePose(new double[]{camPose.getPosition().x, camPose.getPosition().y,
                        yaw});

                lastTimeSeenLimelight = getRuntime();
                turret.faceTo(limePosFace);
                pTelemetry.addData("limePosFace", limePosFace);
                pTelemetry.addData("lime pos x", camPose.getPosition().x);
                pTelemetry.addData("lime pos y", camPose.getPosition().y);
            }
        }

        pTelemetry.addData("x", robotPos.getX(DistanceUnit.INCH));
        pTelemetry.addData("y", robotPos.getY(DistanceUnit.INCH));
        pTelemetry.addData("pinpoint yaw", robotPos.getHeading(AngleUnit.DEGREES));

        turret.loop();

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


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        imu.resetYaw();
    }

    @Override
    public void init_loop() {
        int colorNum = isRed ? 1 : 0;
        id = isRed ? 24 : 20;
        pTelemetry.addData("team", color[colorNum]);
        pTelemetry.addData("is close", isClose);
        pTelemetry.addData("is black board pos", isBlackBoardPos);
        pTelemetry.addData("x", blackboard.get("x"));
        pTelemetry.addData("y", blackboard.get("y"));
        pTelemetry.addData("heading", blackboard.get("heading"));
        pTelemetry.addData("angle", turret.yawTurretEncoder.getAngle180to180());

        pTelemetry.update();

        if (gamepad1.aWasPressed()) isRed = !isRed;
        if (gamepad1.bWasPressed()) isClose = !isClose;
        if (gamepad1.xWasPressed()) isBlackBoardPos = !isBlackBoardPos;

        // reset turret 0
        if (gamepad1.yWasPressed()) {
            turret.yawTurretEncoder.zeroHere();
        }
    }

    @Override
    public void start() {
        super.start();
        et.reset();

        if (isBlackBoardPos && blackboard.get("x") != null && blackboard.get("y") != null && blackboard.get("heading") != null) {
            initPose = new Pose2D(DistanceUnit.INCH, (double) blackboard.get("x"), (double) blackboard.get("y"), AngleUnit.DEGREES, TylerMath.wrap(((double) blackboard.get("heading"))-180, -180, 180));
            imuOffset = ((double) blackboard.get("heading"));
        } else {
            if (!isRed) {
                if (isClose) {
                    initPose = new Pose2D(DistanceUnit.INCH, 63.5, -8.5, AngleUnit.DEGREES, 180);
                } else {
                    initPose = new Pose2D(DistanceUnit.INCH, -63.5, -8.5, AngleUnit.DEGREES, 180);
                }

            } else {
                if (isClose) {
                    initPose = new Pose2D(DistanceUnit.INCH, 63.5, 8.5, AngleUnit.DEGREES, 180);
                } else {
                    initPose = new Pose2D(DistanceUnit.INCH, -63.5, 8.5, AngleUnit.DEGREES, 180);
                }

            }
        }
        if (!isRed) {
            yawOffset = 90;
        } else {
            yawOffset = -90;
        }

        odo.setPose(initPose);
        turret.setGoalColor(isRed);
    }
}