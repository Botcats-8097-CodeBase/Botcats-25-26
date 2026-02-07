//package org.firstinspires.ftc.teamcode.autos;
//
//import com.bylazar.telemetry.JoinedTelemetry;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RobotConstants;
//import org.firstinspires.ftc.teamcode.actions.Action;
//import org.firstinspires.ftc.teamcode.actions.ActionBuilder;
//import org.firstinspires.ftc.teamcode.actions.ActionManager;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "move forward shoot 3 actioned")
//public class MoveForwardShoot6Actioned extends OpMode {
//    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
//
//    int id;
//    boolean isRed = false;
//    String[] color = {"blue", "red"};
//
//    ElapsedTime et = new ElapsedTime();
//
//    IMU.Parameters parameters = new IMU.Parameters(
//            new RevHubOrientationOnRobot(
//                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                    RevHubOrientationOnRobot.UsbFacingDirection.UP
//            )
//    );
//
//    AutoRobot robot = new AutoRobot();
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//
//    public double stripXCoordS = 46;
//    public double stripXCoordE = 24;
//    public double intakeOffset = -5;
//    public double intakeAngle = 180;
//
//    private Pose startPose, scorePose, parkPose, firstStripS, firstStripE, secondStripS, secondStripE, thirdStripS, thirdStripE;
//    private Path scorePreload;
//    private PathChain parkPath, ethanPath1, ethanPath2, ethanPath3;
//
//
//    public void buildPaths() {
//        if (!isRed) {
//            startPose = AutoConstants.blueCloseStartPos;
//            scorePose = new Pose(53.5, 88, Math.toRadians(135));
//            parkPose = new Pose(53.5, 40, Math.toRadians(90));
//        } else {
//            startPose = AutoConstants.redCloseStartPos;
//            scorePose = new Pose(90.5, 88, Math.toRadians(45));
//            parkPose = new Pose(90.5, 40, Math.toRadians(90));
//            stripXCoordS = 98;
//            stripXCoordE = 120;
//            intakeOffset = 5;
//            intakeAngle = 0;
//
//        }
//        firstStripS = new Pose(stripXCoordS, 85, Math.toRadians(intakeAngle));
//        firstStripE = new Pose(stripXCoordE, 85, Math.toRadians(intakeAngle));
//        secondStripS = new Pose(stripXCoordS, 58, Math.toRadians(intakeAngle));
//        secondStripE = new Pose(stripXCoordE, 58, Math.toRadians(intakeAngle));
//        thirdStripS = new Pose(stripXCoordS, 38, Math.toRadians(intakeAngle));
//        if (isRed) stripXCoordE += 2;
//        thirdStripE = new Pose(stripXCoordE-1, 38, Math.toRadians(intakeAngle));
//
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        parkPath = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, parkPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
//                .build();
//
//        ethanPath1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, firstStripS))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), firstStripS.getHeading())
//                .addPath(new BezierLine(firstStripS, firstStripE))
//                .setLinearHeadingInterpolation(firstStripS.getHeading(), firstStripE.getHeading())
//                .addPath(new BezierLine(firstStripE, scorePose.plus(new Pose(0, -4))))
//                .setLinearHeadingInterpolation(firstStripE.getHeading(), scorePose.getHeading())
//                .build();
//
//        ethanPath2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, secondStripS))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), secondStripS.getHeading())
//                .addPath(new BezierLine(secondStripS, secondStripE))
//                .setLinearHeadingInterpolation(secondStripS.getHeading(), secondStripE.getHeading())
//                .addPath(new BezierLine(secondStripE, scorePose.plus(new Pose(0, intakeOffset))))
//                .setLinearHeadingInterpolation(secondStripE.getHeading(), scorePose.getHeading())
//                .build();
//
//        ethanPath3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, thirdStripS))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdStripS.getHeading())
//                .addPath(new BezierLine(thirdStripS, thirdStripE))
//                .setLinearHeadingInterpolation(thirdStripS.getHeading(), thirdStripE.getHeading())
//                .addPath(new BezierLine(thirdStripE, scorePose.plus(new Pose(0, intakeOffset))))
//                .setLinearHeadingInterpolation(thirdStripE.getHeading(), scorePose.getHeading())
//                .build();
//    }
//
//    public Action buildAction() {
//        return new ActionBuilder()
//                .doNow(() -> {
//                    follower.followPath(scorePreload);
//                    robot.turret.goToPreset(RobotConstants.autoSpeedPreset);
//                    telemetry.addData("step", 1);
//                })
//                .waitUntil(() -> !follower.isBusy())
//                .loopFor(t -> {
//                    robot.turret.continueShootSequence(RobotConstants.autoSpeedPreset);
//                    telemetry.addData("step", 2);
//                    autoPitch();
//                }, 4)
//                .loopFor(t -> {
//                    robot.turret.stopIntake();
//                    robot.turret.stopShootSequence();
//                }, 0.5)
//                .doNow(() -> follower.followPath(ethanPath1))
//                .loopUntil(() -> robot.turret.triggerIntake(), () -> follower.isBusy())
//                .doNow(() -> {
//                    robot.turret.stopIntake();
//                })
//                .loopFor(t -> {
//                    robot.turret.continueShootSequence();
//                    autoPitch();
//                }, 4)
//                .loopFor(t -> {
//                    robot.turret.stopIntake();
//                    robot.turret.stopShootSequence();
//                }, 0.5)
//                .doNow(() -> follower.followPath(ethanPath2))
//                .loopUntil(() -> robot.turret.triggerIntake(), () -> follower.isBusy())
//                .doNow(() -> {
//                    robot.turret.stopIntake();
//                })
//                .loopFor(t -> {
//                    robot.turret.continueShootSequence(RobotConstants.autoSpeedPreset);
//                    autoPitch();
//                }, 4)
//                .loopFor(t -> {
//                    robot.turret.stopIntake();
//                    robot.turret.stopShootSequence();
//                }, 0.5)
//                .doNow(() -> follower.followPath(ethanPath3))
//                .loopUntil(() -> robot.turret.triggerIntake(), () -> follower.isBusy())
//                .doNow(() -> {
//                    robot.turret.stopIntake();
//                })
//                .loopFor(t -> {
//                    robot.turret.continueShootSequence(RobotConstants.autoSpeedPreset);
//                    autoPitch();
//                }, 4)
//                .doNow(() -> {
//                    robot.turret.stopIntake();
//                    follower.followPath(parkPath);
//                    robot.turret.stopShootSequence();
//                })
//                .build();
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    public void autoPitch() {
//        robot.turret.autoPitch(0, 0, isRed);
//    }
//
//    @Override
//    public void loop() {
//        double dt = et.milliseconds();
//        et.reset();
//        pTelemetry.addData("dt", dt);
//
//        follower.update();
//        TylerDrawing.draw(follower);
//
//        pTelemetry.addData("turret Target Vel", RobotConstants.autoSpeedPreset[0]);
//        pTelemetry.addData("turret Current Vel", robot.turret.spinnerMotor1.getVelocity());
//        ActionManager.update();
//        telemetry.update();
//
//        robot.update();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        actionTimer = new Timer();
//        et = new ElapsedTime();
//        opmodeTimer.resetTimer();
//
//        robot.init(hardwareMap);
//        robot.turret.setShootPreset(RobotConstants.autoSpeedPreset);
//        robot.turret.useAutoPitch = true;
//
//        follower = Constants.createFollower(hardwareMap);
//
//        ActionManager.schedule(buildAction());
//    }
//
//    @Override
//    public void init_loop() {
//        int colorNum = isRed ? 1 : 0;
//        id = isRed ? 24 : 20;
//        pTelemetry.addData("team", color[colorNum]);
//
//        pTelemetry.update();
//
//        if (gamepad1.aWasPressed()) isRed = !isRed;
//    }
//
//    @Override
//    public void start() {
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void stop() {
//        blackboard.put("x", -follower.getPose().getY() + 72);
//        blackboard.put("y", follower.getPose().getX() - 72);
//        blackboard.put("heading", Math.toDegrees(follower.getPose().getHeading()) - 90);
//    }
//}