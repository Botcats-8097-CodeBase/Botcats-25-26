package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.ActionBuilder;
import org.firstinspires.ftc.teamcode.actions.ActionManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConversion;

@Autonomous(name = "12 auto")
public class Auto12Actioned2 extends OpMode {
    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    int id;
    boolean isRed = false;
    boolean isClose = true;
    String[] color = {"blue", "red"};
    ElapsedTime et = new ElapsedTime();

    AutoRobot robot = new AutoRobot();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public double stripXCoordS = 46;
    public double stripXCoordE = 24;
    public double intakeOffset = -4;
    public double intakeAngle = 180;

    private Pose startPose, scorePose, parkPose, firstStripS, firstStripE, secondStripS, secondStripE, thirdStripS, thirdStripE;
    private Path scorePreload;
    private PathChain parkPath, ethanPath1, ethanPath2, ethanPath3;
    public void buildPaths() {
        if (!isRed) {
            if (isClose)
                startPose = AutoConstants.blueCloseStartPos;
            else
                startPose = AutoConstants.blueFarStartPos;
            scorePose = new Pose(53.5, 88, Math.toRadians(135));
            parkPose = new Pose(53.5, 40, Math.toRadians(90));
        } else {
            if (isClose)
                startPose = AutoConstants.redCloseStartPos;
            else
                startPose = AutoConstants.redFarStartPos;
            scorePose = new Pose(90.5, 88, Math.toRadians(45));
            parkPose = new Pose(90.5, 40, Math.toRadians(90));
            stripXCoordS = 98;
            stripXCoordE = 120;
            intakeOffset = 4;
            intakeAngle = 0;

            robot.turret.presetOffset = new double[]{0.0, 0.1};
        }


        firstStripS = new Pose(stripXCoordS, 85, Math.toRadians(intakeAngle));
        firstStripE = new Pose(stripXCoordE, 85, Math.toRadians(intakeAngle));
        secondStripS = new Pose(stripXCoordS, 58, Math.toRadians(intakeAngle));
        secondStripE = new Pose(stripXCoordE + intakeOffset, 58, Math.toRadians(intakeAngle));
        thirdStripS = new Pose(stripXCoordS, 38, Math.toRadians(intakeAngle));
        thirdStripE = new Pose(stripXCoordE + intakeOffset, 38, Math.toRadians(intakeAngle));

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

        ethanPath1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, firstStripS))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstStripS.getHeading())
                .addPath(new BezierLine(firstStripS, firstStripE))
                .setLinearHeadingInterpolation(firstStripS.getHeading(), firstStripE.getHeading())
                .addPath(new BezierLine(firstStripE, scorePose))
                .setLinearHeadingInterpolation(firstStripE.getHeading(), scorePose.getHeading())
                .setBrakingStart(0.1)
                .build();

        ethanPath2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondStripS))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondStripS.getHeading())
                .addPath(new BezierLine(secondStripS, secondStripE))
                .setLinearHeadingInterpolation(secondStripS.getHeading(), secondStripE.getHeading())
                .addPath(new BezierLine(secondStripE, scorePose))
                .setLinearHeadingInterpolation(secondStripE.getHeading(), scorePose.getHeading())
                .build();

        ethanPath3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, thirdStripS))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdStripS.getHeading())
                .addPath(new BezierLine(thirdStripS, thirdStripE))
                .setLinearHeadingInterpolation(thirdStripS.getHeading(), thirdStripE.getHeading())
                .addPath(new BezierLine(thirdStripE, scorePose))
                .setLinearHeadingInterpolation(thirdStripE.getHeading(), scorePose.getHeading())
                .build();
    }

    public Action buildAction() {
        return new ActionBuilder()
                .doNow(() -> {
                    follower.followPath(scorePreload);
                    robot.turret.spinUp();
                })
                .waitUntil(() -> !follower.isBusy())
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 3.5)
                .loopFor(t -> {
                    robot.turret.stopIntake();
                    robot.turret.stopShootSequence();
                }, 1)
                .doNow(() -> follower.followPath(ethanPath1))
                .loopUntil(() -> {
                    robot.turret.triggerIntake();
                    robot.turret.spinUp();
                }, () -> !follower.isBusy())
                .doNow(() -> {
                    robot.turret.stopIntake();
                    actionTimer.resetTimer();
                })
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 3.5)
                .loopFor(t -> {
                    robot.turret.stopIntake();
                    robot.turret.stopShootSequence();
                }, 1)
                .doNow(() -> follower.followPath(ethanPath2))
                .loopUntil(() -> {
                    robot.turret.triggerIntake();
                    robot.turret.spinUp();
                }, () -> !follower.isBusy())
                .doNow(() -> {
                    robot.turret.stopIntake();
                    actionTimer.resetTimer();
                })
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 3.5)
                .loopFor(t -> {
                    robot.turret.stopIntake();
                    robot.turret.stopShootSequence();
                }, 1)
                .doNow(() -> follower.followPath(ethanPath3))
                .loopUntil(() -> {
                    robot.turret.triggerIntake();
                    robot.turret.spinUp();
                }, () -> !follower.isBusy())
                .doNow(() -> {
                    robot.turret.stopIntake();
                    actionTimer.resetTimer();
                })
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 3.5)
                .doNow(() -> {
                    robot.turret.stopIntake();
                    follower.followPath(parkPath);
                    robot.turret.stopShootSequence();
                })
                .build();

    }

    @Override
    public void loop() {

        follower.update();
        TylerDrawing.draw(follower);

        robot.turret.updatePose(
                PedroConversion.pedroToOdo(
                        new double[]{follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()}
                )
        );
        robot.update();

        pTelemetry.addData("turret Current Vel", robot.turret.spinnerMotor1.getVelocity());
        pTelemetry.addData("path state", pathState);

        ActionManager.update();

        pTelemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        robot.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        robot.turret.pTelemetry = pTelemetry;

        ActionManager.schedule(buildAction());
    }

    @Override
    public void init_loop() {
        int colorNum = isRed ? 1 : 0;
        id = isRed ? 24 : 20;
        pTelemetry.addData("team", color[colorNum]);
        pTelemetry.addData("isClose", isClose);

        pTelemetry.update();

        if (gamepad1.aWasPressed()) isRed = !isRed;
        if (gamepad1.bWasPressed()) isClose = !isClose;
    }

    @Override
    public void start() {
        buildPaths();
        follower.setStartingPose(startPose);

        opmodeTimer.resetTimer();
    }

    @Override
    public void stop() {
        double[] con = PedroConversion.pedroToOdo(new double[]{follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()});
        blackboard.put("x", con[0]);
        blackboard.put("y", con[1]);
        blackboard.put("heading", con[2]);
    }
}