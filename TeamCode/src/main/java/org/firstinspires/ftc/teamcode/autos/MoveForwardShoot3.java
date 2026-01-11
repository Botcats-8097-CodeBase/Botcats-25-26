package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "move forward shoot 3")
public class MoveForwardShoot3 extends OpMode {
    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    int id;
    boolean isRed = false;
    boolean isClose = true;
    String[] color = {"blue", "red"};

    ElapsedTime et = new ElapsedTime();

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    AutoRobot robot = new AutoRobot();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Pose startPose, scorePose, parkPose;
    private Path scorePreload;
    private PathChain parkPath, shakePath, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    public void buildPaths() {
        if (!isRed) {
            startPose = AutoConstants.blueCloseStartPos;
            scorePose = new Pose(53.5, 90, Math.toRadians(135));
            parkPose = new Pose(53.5, 40, Math.toRadians(90));
        } else {
            startPose = AutoConstants.redCloseStartPos;
            scorePose = new Pose(90.5, 90, Math.toRadians(45));
            parkPose = new Pose(90.5, 40, Math.toRadians(90));
        }
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

        shakePath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, scorePose.plus(new Pose(0, 5, 0))))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
                .addPath(new BezierLine(scorePose.plus(new Pose(0, 5, 0)), scorePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    setPathState(2);
                    actionTimer.resetTimer();
                }
                break;
            case 2:
                robot.turret.continueShootSequence(RobotConstants.autoSpeedPreset);
                if (actionTimer.getElapsedTime() > 9000) {
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(parkPath);
                setPathState(5);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        TylerDrawing.draw(follower);

        pTelemetry.addData("turret Target Vel", RobotConstants.autoSpeedPreset[0]);
        pTelemetry.addData("turret Current Vel", robot.turret.spinnerMotor1.getVelocity());

        Double limeFacingTarget = robot.limelight.limeAutoFacing(robot.turret.getCurrentFacing(), id);
        Pose3D camPose = robot.limelight.limePosFace();
        if (limeFacingTarget != null && camPose != null) {
            double limePosFace = robot.turret.autoFace(camPose.getPosition().x * 39.37, camPose.getPosition().y * 39.37,
                    Math.toDegrees(follower.getPose().getHeading()) - 90, isRed);

            robot.turret.faceTo(limePosFace);
            pTelemetry.addData("limePosFace", limePosFace);
            pTelemetry.addData("lime pos x", camPose.getPosition().x * 39.37);
            pTelemetry.addData("lime pos y", camPose.getPosition().y * 39.37);
        }

        telemetry.addData("path state", pathState);
        telemetry.update();

        robot.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        robot.init(hardwareMap, isRed);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        int colorNum = isRed ? 1 : 0;
        id = isRed ? 24 : 20;
        pTelemetry.addData("team", color[colorNum]);
        pTelemetry.addData("is close", isClose);

        pTelemetry.update();

        if (gamepad1.aWasPressed()) isRed = !isRed;
        if (gamepad1.bWasPressed()) isClose = !isClose;
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        blackboard.put("x", -follower.getPose().getY() + 72);
        blackboard.put("y", -follower.getPose().getX() + 72);
        blackboard.put("heading", Math.toDegrees(follower.getPose().getHeading()) + 90);
    }
}