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

@Autonomous(name = "12 had")
public class HAD12 extends OpMode {
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
    private Path scorePreload;
    private PathChain parkPath, ethanPath1A, ethanPath1B, ethanPath2A, ethanPath2B, ethanPath3A, ethanPath3B;
    private PathChain ethanPath1RA, ethanPath1RB, ethanPath2RA, ethanPath2RB, ethanPath3RA, ethanPath3RB;
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

        ethanPath1RA = follower.pathBuilder()
            .addPath(
                new BezierCurve(
                    scorePose,
                    new Pose(58.407, 99.663),
                    new Pose(14.500, 98.000)
                )
            ).setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(300))
            .addPath(
                new BezierCurve(
                    new Pose(14.500, 98.000),
                    new Pose(26.215, 83.349),
                    new Pose(36.349, 74.907)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(0))
            .build();

        ethanPath1RB = follower.pathBuilder()
            .addPath(new BezierLine(new Pose(36.349, 74.907), scorePose))
            .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
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

        ethanPath2RA = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(66.326, 12.779),
                                new Pose(11.500, 22.500)
                        )
                ).setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(250))
                .addPath(
                        new BezierLine(
                                new Pose(11.500, 22.500),

                                new Pose(11.512, 8.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(270))

                .build();

        ethanPath2RB = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.512, 8.500),
                                new Pose(70.116, 7.052),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), scorePose.getHeading())
                .build();

        ethanPath3A = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, thirdStripS))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdStripS.getHeading())
                .addPath(new BezierLine(thirdStripS, thirdStripE))
                .setLinearHeadingInterpolation(thirdStripS.getHeading(), thirdStripE.getHeading())
                .build();

        ethanPath3B = follower.pathBuilder()
                .addPath(new BezierLine(thirdStripE, thirdStripS))
                .setLinearHeadingInterpolation(thirdStripE.getHeading(), thirdStripS.getHeading())
                .addPath(new BezierLine(thirdStripS, scorePose))
                .setLinearHeadingInterpolation(thirdStripS.getHeading(), scorePose.getHeading())
                .build();

        ethanPath3RA = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                            scorePose,
                            new Pose(67.000, 25.000),
                            new Pose(11.000, 61.500),
                            new Pose(7.5, 37.5)
                    )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(7.5, 37.5), thirdStripS))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ethanPath3RB = follower.pathBuilder()
                .addPath(new BezierLine(thirdStripS, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
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



                .doNow(() -> {
                    robot.turret.stopShootSequence();
                    follower.followPath(ethanPath1RA);
                    robot.turret.triggerIntake();
                })
                .waitUntil(() -> !follower.isBusy())
                .doNow(() -> {
                    robot.turret.stopIntake();
                    follower.followPath(ethanPath1RB);
                })
                .waitUntil(() -> !follower.isBusy())
                .loopFor(t -> {
                    robot.turret.continueShootSequence();
                }, 3.5)



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
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        robot.turret.pTelemetry = pTelemetry;
        robot.turret.shootOverride = 0.4;

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