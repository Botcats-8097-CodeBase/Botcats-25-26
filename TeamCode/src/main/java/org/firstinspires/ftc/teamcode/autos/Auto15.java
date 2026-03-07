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

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.ActionBuilder;
import org.firstinspires.ftc.teamcode.actions.ActionManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConversion;

@Autonomous(name = "15 auto")
public class Auto15 extends OpMode {
    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    int id;
    boolean isRed = false;
    boolean isClose = true;
    String[] color = {"blue", "red"};
    ElapsedTime et = new ElapsedTime();

    AutoRobot robot = new AutoRobot();

    private Follower follower;
    private Timer opmodeTimer;
    private int pathState;

    public double stripXCoordS = 46;
    public double stripXCoordE = 24;
    public double intakeOffset = -4;
    public double intakeAngle = 180;

    private Pose startPose, scorePose, parkPose, firstStripS, firstStripE, secondStripS, secondStripE, thirdStripS, thirdStripE;
    private Pose transativeMPose, transativeEPose;
    private Path scorePreload;
    private PathChain parkPath, ethanPath1A, ethanPath1B, ethanPath2A, ethanPath2B, ethanPath3A, ethanPath3B, ethanPath3RA;
    private PathChain transitivePath1, transitivePath2, transitivePath15;
    public void buildPaths() {
        if (!isRed) {
            if (isClose)
                startPose = AutoConstants.blueCloseStartPos;
            else
                startPose = AutoConstants.blueFarStartPos;
            transativeEPose = new Pose(8, 63, Math.toRadians(155));
            scorePose = new Pose(53.5, 88, Math.toRadians(135));
            parkPose = new Pose(53.5, 40, Math.toRadians(90));
        } else {
            if (isClose)
                startPose = AutoConstants.redCloseStartPos;
            else
                startPose = AutoConstants.redFarStartPos;
            scorePose = new Pose(90.5, 88, Math.toRadians(45));
            parkPose = new Pose(90.5, 40, Math.toRadians(90));
            transativeEPose = new Pose(8, 63, Math.toRadians(155)).mirror();
            stripXCoordS = 98;
            stripXCoordE = 120;
            intakeOffset = 4;
            intakeAngle = 0;

            robot.turret.presetOffset = new double[]{-0.2, 0.0};
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

        ethanPath1A = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                (!isRed) ? new Pose(36.700, 86.000) : new Pose(36.700, 86.000).mirror(),
                                firstStripE
                        )
                )
                .build();

        ethanPath1B = follower.pathBuilder()
                .addPath(new BezierLine(firstStripE, scorePose))
                .setLinearHeadingInterpolation(firstStripE.getHeading(), scorePose.getHeading())
                .setBrakingStart(1)
                .build();

        ethanPath2A = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                (!isRed) ? new Pose(55.5, 48) : new Pose(55.5, 48).mirror(),
                                secondStripE
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondStripE.getHeading())
                .setGlobalDeceleration(1)
                .build();

        ethanPath2B = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                secondStripE,
                                (!isRed) ? new Pose(55.5, 48) : new Pose(55.5, 48).mirror(),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(secondStripE.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration(1)
                .build();


        ethanPath3A = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, thirdStripS))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdStripS.getHeading())
                .addPath(new BezierLine(thirdStripS, thirdStripE))
                .setLinearHeadingInterpolation(thirdStripS.getHeading(), thirdStripE.getHeading())
                .build();

        ethanPath3B = follower.pathBuilder()
                .addPath(new BezierLine(thirdStripE, scorePose))
                .setLinearHeadingInterpolation(thirdStripE.getHeading(), scorePose.getHeading())
                .build();

        transitivePath1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                (!isRed) ? new Pose(52.5, 61.5) : new Pose(52.5, 61.5).mirror(),
                                transativeEPose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), transativeEPose.getHeading())
                .setGlobalDeceleration(1)
                .build();

        transitivePath2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                transativeEPose,
                                (!isRed) ? new Pose(52.5, 61.5) : new Pose(52.5, 61.5).mirror(),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(transativeEPose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration(1)
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
                }, 2.5)



                .doNow(() -> {
                    robot.turret.stopShootSequence();
                    follower.followPath(ethanPath2A);
                    robot.turret.triggerIntake();
                })
                .waitUntil(() -> !follower.isBusy())
                .doNow(() -> {
                    robot.turret.stopIntake();
                    follower.followPath(ethanPath2B);
                })
                .waitUntil(() -> !follower.isBusy())
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 1.7)



                .doNow(() -> {
                    robot.turret.stopShootSequence();
                    follower.followPath(transitivePath1);
                    robot.turret.triggerIntake();
                })
                .waitUntil(() -> !follower.isBusy())
                .wait(1.0)
                .doNow(() -> {
                    robot.turret.stopIntake();
                    follower.followPath(transitivePath2);
                })                .waitUntil(() -> !follower.isBusy())
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 1.7)




                .doNow(() -> {
                    robot.turret.stopShootSequence();
                    follower.followPath(ethanPath1A);
                    robot.turret.triggerIntake();
                })
                .waitUntil(() -> !follower.isBusy())
                .doNow(() -> {
                    robot.turret.stopIntake();
                    follower.followPath(ethanPath1B);
                })
                .waitUntil(() -> !follower.isBusy())
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 1.7)



                .doNow(() -> {
                    robot.turret.stopShootSequence();
                    follower.followPath(ethanPath3A);
                    robot.turret.triggerIntake();
                })
                .waitUntil(() -> !follower.isBusy())
                .doNow(() -> {
                    robot.turret.stopIntake();
                    follower.followPath(ethanPath3B);
                })
                .waitUntil(() -> !follower.isBusy())
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 1.7)



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
        opmodeTimer = new Timer();
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
        blackboard.put("yawPos", robot.turret.getYawPos0to360());
    }
}