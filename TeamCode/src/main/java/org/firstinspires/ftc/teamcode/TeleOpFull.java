package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import org.firstinspires.ftc.teamcode.utils.TylerMath;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import org.firstinspires.ftc.teamcode.subcomponents.Intake;
import org.firstinspires.ftc.teamcode.subcomponents.Limelight;
import org.firstinspires.ftc.teamcode.subcomponents.Odometry;
import org.firstinspires.ftc.teamcode.subcomponents.Turret;
import org.firstinspires.ftc.teamcode.utils.BasicRobot;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@TeleOp(name = "!teleopfull")
public class TeleOpFull extends OpMode {

    List<LynxModule> allHubs;

    TelemetryManager.TelemetryWrapper pTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    BasicRobot robot = new BasicRobot();
    Intake intake = new Intake();
    Turret turret = new Turret();
    Limelight limelight = new Limelight();

    Odometry odo = new Odometry();

    boolean isRed = false;
    String[] color = {"blue", "red"};

    boolean isClose = true;
    boolean isBlackBoardPos = true;

    boolean isConstantPreset = false;

    double yawOffset = 90;

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

    double[] preset = RobotConstants.fullSpeedPreset;



    @Override
    public void start() {
        super.start();
        et.reset();

        odo.init(hardwareMap, initPose);
    }

    @Override
    public void loop() {
        float dt = (float) et.milliseconds();
        et.reset();

        for (LynxModule hub : allHubs) hub.clearBulkCache();

        // reset IMU
        if (gamepad1.y) {
            imu.resetYaw();
            imu.initialize(parameters);
        }

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        pTelemetry.addData("Yaw", yaw);

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

        if (gamepad1.right_bumper || gamepad2.right_bumper) intake.trigger();
        else if (gamepad1.left_bumper || gamepad2.left_bumper) intake.reverseTrigger();
        else intake.stop();

        if (gamepad2.a) preset = RobotConstants.fullSpeedPreset;
        if (gamepad2.b) preset = RobotConstants.closestSpeedPreset;

        if (gamepad1.backWasPressed()) isAutoAiming = !isAutoAiming;

        pTelemetry.addData("is auto aiming", isAutoAiming);

        if (gamepad1.dpadLeftWasPressed()) isConstantPreset = !isConstantPreset;

        if (gamepad1.a) turret.continueShootSequence(preset);
        else if (isConstantPreset) turret.goToPreset(preset);
        else turret.stopShootSequence();

        pTelemetry.addData("turret Vel", turret.spinnerMotor1.getVelocity());
        pTelemetry.addData("turret Pwr", turret.spinnerMotor1.getPower());

        odo.update();
        Pose2D robotPos = odo.getPose();



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
            if (gamepad1.dpad_down) RobotConstants.yawTurretStartEncoder += 1;
            if (gamepad1.dpad_up) RobotConstants.yawTurretStartEncoder -= 1;

            double facingTarget = autoFace(robotPos.getX(DistanceUnit.INCH), robotPos.getY(DistanceUnit.INCH), yaw);
            facingTarget = limelight.limeAutoFacing(facingTarget, id);
            facingTarget = TylerMath.normalize180(facingTarget);
            facingTarget = clip(facingTarget, RobotConstants.yawTurretMinAngle, RobotConstants.yawTurretMaxAngle);
            turret.faceTo(facingTarget);

            pTelemetry.addData("turret Target Vel", preset[0]);
            pTelemetry.addData("turret Current Vel", turret.spinnerMotor1.getVelocity());
            pTelemetry.addData("turret Current Pwr", turret.spinnerMotor1.getPower());
            pTelemetry.addData("facingTarget", facingTarget);
        }

        pTelemetry.addData("x", robotPos.getX(DistanceUnit.INCH));
        pTelemetry.addData("y", robotPos.getY(DistanceUnit.INCH));

        turret.loop();

        pTelemetry.update();

    }

    public double autoFace(double x, double y, double yaw) {
        return 0;
//        double gx;
//        double gy;
//        if (isRed) {
//            gx = -72;
//            gy = 72;
//        } else {
//            gx = -72;
//            gy = -72;
//        }
//        double out = TylerMath.wrap(-Math.toDegrees(Math.atan2(gy - y, gx - x)) + yaw + 180, 0, 360);
//
//        // Bounds 0~360 to -180~180
//        if (out > 180) out -= 360;
//
//        return out;
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        limelight.init(hardwareMap);


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
        pTelemetry.addData("angle", turret.yawTurretEncoder.getAngle180to180());

        pTelemetry.update();

        if (gamepad1.aWasPressed()) isRed = !isRed;
        if (gamepad1.bWasPressed()) isClose = !isClose;
        if (gamepad1.xWasPressed()) isBlackBoardPos = !isBlackBoardPos;

        // reset turret 0
        if (gamepad1.yWasPressed()) {
            turret.yawTurretEncoder.zeroHere();
        }

        if (isBlackBoardPos && blackboard.get("x") != null && blackboard.get("y") != null) {
            initPose = new Pose2D(DistanceUnit.INCH, (double) blackboard.get("x"), (double) blackboard.get("y"), AngleUnit.DEGREES, 180);
        } else {
            if (!isRed) {
                if (isClose) {
                    initPose = new Pose2D(DistanceUnit.INCH, 63.5, -8.5, AngleUnit.DEGREES, 180);
                } else {
                    initPose = new Pose2D(DistanceUnit.INCH, -63.5, -8.5, AngleUnit.DEGREES, 180);
                }
                yawOffset = 90;
            } else {
                if (isClose) {
                    initPose = new Pose2D(DistanceUnit.INCH, 63.5, 8.5, AngleUnit.DEGREES, 180);
                } else {
                    initPose = new Pose2D(DistanceUnit.INCH, -63.5, 8.5, AngleUnit.DEGREES, 180);
                }
                yawOffset = -90;
            }
        }
    }
}