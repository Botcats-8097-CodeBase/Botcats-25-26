package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import static org.firstinspires.ftc.teamcode.utils.TylerMath.normalize180;

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

    double adjustAngle = 0;
    double filteredTx = 0;
    double lastVisionTarget = 0;

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
    public void init() {
        initRobot();
    }

    @Override
    public void start() {
        super.start();
        et.reset();

        odo.init(hardwareMap, initPose);
//        if (isBlackBoardPos && blackboard.get("yawEncoder") != null) {
//            RobotConstants.yawTurretStartEncoder = (int) blackboard.get("yawEncoder");
//        }
    }

    @Override
    public void loop() {
        float dt = (float) et.milliseconds();
        et.reset();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // reset IMU
        if (gamepad1.y) {
            imu.resetYaw();
            imu.initialize(parameters);
//            odo.driver.recalibrateIMU();
        }



        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
//        double yaw = odo.driver.getHeading(AngleUnit.DEGREES);//robotOrientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw", yaw);

        // pressurising the right trigger slows down the drive train
        double coefficient = 0.35;
        if(gamepad1.right_trigger < 0.5)
        {
            telemetry.addData("Speed Mode", "off");
        }
        else
        {
            telemetry.addData("Speed Mode", "on");
            coefficient = 1;
        }

        robot.drivePower(
                new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y)
                        .rotateVector(-Math.toRadians(yaw-yawOffset))
                , gamepad1.right_stick_x, coefficient);

        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intake.trigger();
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
            intake.reverseTrigger();
        } else {
            intake.stop();
        }

        if (gamepad2.a) {
            preset = RobotConstants.fullSpeedPreset;
        }
        if (gamepad2.b) {
            preset = RobotConstants.closestSpeedPreset;
        }

        if (gamepad1.backWasPressed()) {
            isAutoAiming = !isAutoAiming;
        }
        telemetry.addData("is auto aiming", isAutoAiming);

        if (gamepad1.dpadLeftWasPressed()) {
            isConstantPreset = !isConstantPreset;
        }
        if (gamepad1.a) {
            turret.continueShootSequence(preset);
        } else if (isConstantPreset) {
            turret.goToPreset(preset);
        } else {
            turret.stopShootSequence();
        }
        telemetry.addData("turret Vel", turret.spinnerMotor.getVelocity());
        telemetry.addData("turret Pwr", turret.spinnerMotor.getPower());
//        telemetry.addData("turret Yaw Current", turret.yawTurretMotor.getCurrentPower());

        odo.update();
        Pose2D robotPos = odo.getPose();

        if (!isAutoAiming) {
            if (gamepad1.dpad_down) {
                targetTurretAngle -= 1;
            }
            if (gamepad1.dpad_up) {
                targetTurretAngle += 1;
            }

            telemetry.addData("targetTurretAngle", targetTurretAngle);

            turret.faceTo(targetTurretAngle);

            if (gamepad2.yWasPressed()) {
                targetTurretAngle = 0;
                isAutoAiming = true;
            }
        } else {
            if (gamepad1.dpad_down) {
                adjustAngle -= 1;
            }
            if (gamepad1.dpad_up) {
                adjustAngle += 1;
            }

            telemetry.addData("turret Target Vel", preset[0]);
            telemetry.addData("turret Current Vel", turret.spinnerMotor.getVelocity());
            telemetry.addData("turret Current Pwr", turret.spinnerMotor.getPower());
            telemetry.addData("turret is stopped", turret.spinnerMotor.isStopped());
            telemetry.addData("adjust Angle", adjustAngle);


            double targ2 = autoFace(robotPos.getX(DistanceUnit.INCH), robotPos.getY(DistanceUnit.INCH), yaw);



//            if (Double.isNaN(tx)) {
//                turret.faceTo(0);
//            }


            double tx = limelight.limeScanPosX(id);
            telemetry.addData("tx", tx);
            if (!Double.isNaN(tx)) {

                double alpha = 0.3;
                filteredTx = filteredTx + alpha * (tx - filteredTx);

                double turretAngle = turret.getCurrentFacing();
                double kVision = 1.0;

                double newAngle = normalize180(turretAngle + kVision * filteredTx);
                newAngle = clip(newAngle, RobotConstants.yawTurretMinAngle, RobotConstants.yawTurretMaxAngle);

                lastVisionTarget = newAngle;
                turret.faceTo(newAngle);
                telemetry.addData("targeting", newAngle);
            }
            else {
                turret.faceTo(targ2); //turret.faceTo(targ2 + adjustAngle);
            }
            telemetry.addData("targ2", targ2);
        }

        telemetry.addData("x", robotPos.getX(DistanceUnit.INCH));
        telemetry.addData("y", robotPos.getY(DistanceUnit.INCH));

        turret.loop();

        telemetry.update();

    }


    public double autoFace(double x, double y, double yaw) {
        double gx;
        double gy;
        if (isRed) {
            gx = -72;
            gy = 72;
        } else {
            gx = -72;
            gy = -72;
        }
        double out = ((-Math.toDegrees(Math.atan2(gy - y, gx - x)) + yaw + 180) % 360 + 360) % 360;

        // Bounds 0~360 to -180~180
        if (out > 180) out -= 360;

        return out;
    }

    void initRobot() {
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
        telemetry.addData("team", color[colorNum]);
        telemetry.addData("is close", isClose);
        telemetry.addData("is black board pos", isBlackBoardPos);
        telemetry.addData("x", blackboard.get("x"));
        telemetry.addData("y", blackboard.get("y"));

        //telemetry.addData("motor", turret.yawTurretMotor.getCurrentPosition());
        telemetry.addData("angle", turret.yawTurretEncoder.getAngle180to180());

        telemetry.update();

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