package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConversion;

@Autonomous(name = "12 pof")
public class POF12 extends OpMode {
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

    public double stripXCordS = 46;
    public double stripXCordE = 20;
    public double intakeOffset = -8;
    public double intakeAngle = 180;

    public double shootTimeMs = 2500;

    private Pose startPose, scorePose, parkPose, firstStripS, firstStripE, secondStripS, secondStripE, thirdStripS, thirdStripE, transativeMPose, transativeEPose;
    private Path scorePreload;
    private PathChain parkPath, ethanPath1, ethanPath2, ethanPath3, transitivePath1, transitivePath2, transitivePath15;
    public void buildPaths() {
        if (!isRed) {
            if (isClose)
                startPose = AutoConstants.blueCloseStartPos;
            else
                startPose = AutoConstants.blueFarStartPos;
            scorePose = new Pose(56.5, 84, Math.toRadians(140));
            parkPose = new Pose(53.5, 40, Math.toRadians(90));

            transativeEPose = new Pose(8, 58, Math.toRadians(150));
            transativeMPose = new Pose(11, 60, Math.toRadians(180));
            robot.turret.presetOffset = new double[]{-0.15, 0};
        } else {
            if (isClose)
                startPose = AutoConstants.redCloseStartPos;
            else
                startPose = AutoConstants.redFarStartPos;
            scorePose = new Pose(90.5, 88, Math.toRadians(40));
            parkPose = new Pose(90.5, 40, Math.toRadians(90));
            stripXCordS = 98;
            stripXCordE = 120;
            intakeOffset = 8;
            intakeAngle = 0;

            robot.turret.presetOffset = new double[]{0.0, 0.1};
        }


        firstStripS = new Pose(stripXCordS, 85, Math.toRadians(intakeAngle));
        firstStripE = new Pose(stripXCordE, 85, Math.toRadians(intakeAngle));
        secondStripS = new Pose(stripXCordS, 56, Math.toRadians(intakeAngle));
        secondStripE = new Pose(stripXCordE + intakeOffset, 56, Math.toRadians(intakeAngle));
        thirdStripS = new Pose(stripXCordS, 38, Math.toRadians(intakeAngle));
        thirdStripE = new Pose(stripXCordE + intakeOffset, 38, Math.toRadians(intakeAngle));

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
                .setBrakingStart(1)
                .build();

        ethanPath2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(55.5, 48),
                                secondStripE
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondStripE.getHeading())
                .addPath(
                        new BezierCurve(
                                secondStripE,
                                new Pose(55.5, 48),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(secondStripE.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration(1)
                .build();

        ethanPath3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, thirdStripS))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdStripS.getHeading())
                .addPath(new BezierLine(thirdStripS, thirdStripE))
                .setLinearHeadingInterpolation(thirdStripS.getHeading(), thirdStripE.getHeading())
                .addPath(new BezierLine(thirdStripE, scorePose))
                .setLinearHeadingInterpolation(thirdStripE.getHeading(), scorePose.getHeading())
                .build();

        transitivePath1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(52.5, 61.5),
                                transativeEPose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), transativeEPose.getHeading())
                .setGlobalDeceleration(1)
                .build();

        transitivePath2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                transativeMPose,
                                new Pose(52.5, 61.5),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(transativeMPose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration(1)
                .build();

        transitivePath15 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                transativeEPose,
                                new Pose(9, 58),
                                transativeMPose
                        )
                )
                .setLinearHeadingInterpolation(transativeEPose.getHeading(), transativeMPose.getHeading())
                .setGlobalDeceleration(1)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
            follower.followPath(scorePreload);
            robot.turret.spinUp();
            setPathState(1);
            break;
            case 1:
                if(!follower.isBusy()) {
                    setPathState(2);
                    actionTimer.resetTimer();
                }
                break;
            case 2:
                robot.turret.continueShootSequence();
                if (actionTimer.getElapsedTime() > shootTimeMs) {
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                robot.turret.stopIntake();
                robot.turret.stopShootSequence();
                if (actionTimer.getElapsedTime() > 300) {
                    follower.followPath(ethanPath2, 0.85, true);
                    setPathState(4);
                }
                break;
            case 4:
                robot.turret.triggerIntake();
                robot.turret.spinUp();

                if (!follower.isBusy()) {
                    setPathState(5);
                    robot.turret.stopIntake();
                    actionTimer.resetTimer();
                }
                break;
            case 5:
                robot.turret.continueShootSequence();
                if (actionTimer.getElapsedTime() > shootTimeMs) {
                    actionTimer.resetTimer();
                    robot.turret.triggerIntake();
                    robot.turret.stopShootSequence();
                    follower.followPath(transitivePath1, 0.85, true);
                    setPathState(7);
                }
                break;
// todo START HERE PASTE
            case 7:
                robot.turret.triggerIntake();
                if (!follower.isBusy()) {
                    setPathState(8);
                    actionTimer.resetTimer();
                }
                break;
            case 8:
                if (actionTimer.getElapsedTime() > 1500) {
                    setPathState(9);
                    follower.followPath(transitivePath15);
                    robot.turret.stopIntake();
                    actionTimer.resetTimer();
                }
                break;
            case 9:
                if (actionTimer.getElapsedTime() > 1500 && !follower.isBusy()) {
                    setPathState(10);
                    follower.followPath(transitivePath2);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                robot.turret.continueShootSequence();
                if (actionTimer.getElapsedTime() > shootTimeMs) {
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                robot.turret.stopIntake();
                robot.turret.stopShootSequence();
                follower.followPath(parkPath);
                setPathState(13);
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

        robot.turret.updatePose(
                PedroConversion.pedroToOdo(
                        new double[]{follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()}
                )
        );
        robot.update();

        pTelemetry.addData("turret Current Vel", robot.turret.spinnerMotor1.getVelocity());
        pTelemetry.addData("path state", pathState);
        pTelemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        robot.init(hardwareMap);
//        robot.turret.useDistError(false);

        follower = Constants.createFollower(hardwareMap);

        robot.turret.pTelemetry = pTelemetry;
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
        setPathState(0);
    }

    @Override
    public void stop() {
        double[] con = PedroConversion.pedroToOdo(new double[]{follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()});
        blackboard.put("x", con[0] - 2.2);
        blackboard.put("y", con[1] - 0.2);
        blackboard.put("heading", con[2]);
        blackboard.put("yawPos", robot.turret.getYawPos0to360());
    }
}