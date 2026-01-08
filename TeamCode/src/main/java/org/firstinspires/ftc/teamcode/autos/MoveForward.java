//package org.firstinspires.ftc.teamcode.autos;
//
//import com.bylazar.telemetry.JoinedTelemetry;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
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
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "move forward")
//public class MoveForward extends OpMode {
//    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
//
//    int id;
//    boolean isRed = false;
//    boolean isClose = true;
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
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//
//    private Pose startPose, scorePose;
//    private Path scorePreload;
//    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
//    public void buildPaths() {
//        if (!isRed) {
//            if (isClose) {
//                startPose = AutoConstants.blueCloseStartPos;
//                scorePose = new Pose(63.5, 40, Math.toRadians(90));
//            } else {
//                startPose = AutoConstants.blueFarStartPos;
//                scorePose = new Pose(63.5, 57, Math.toRadians(90));
//            }
//        } else {
//            if (isClose) {
//                startPose = AutoConstants.redCloseStartPos;
//                scorePose = new Pose(80.5, 40, Math.toRadians(90));
//            } else {
//                startPose = AutoConstants.redFarStartPos;
//                scorePose = new Pose(80.5, 57, Math.toRadians(90));
//            }
//        }
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(scorePreload);
//                setPathState(1);
//                break;
//            case 1:
//                if(!follower.isBusy()) {
//                    setPathState(2);
//                    actionTimer.resetTimer();
//                }
//                break;
//            case 2:
//                if (actionTimer.getElapsedTime() > 1000) {
//                    blackboard.put("x", -follower.getPose().getY() + 72);
//                    blackboard.put("y", -follower.getPose().getX() + 72);
//                    blackboard.put("heading", Math.toDegrees(follower.getPose().getHeading()) + 90);
//                    telemetry.addData("Done", true);
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//
//        follower.update();
//        autonomousPathUpdate();
//        TylerDrawing.draw(follower);
//
//        telemetry.addData("path state", pathState);
//        telemetry.update();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public void init_loop() {
//        int colorNum = isRed ? 1 : 0;
//        id = isRed ? 24 : 20;
//        pTelemetry.addData("team", color[colorNum]);
//        pTelemetry.addData("is close", isClose);
//
//        pTelemetry.update();
//
//        if (gamepad1.aWasPressed()) isRed = !isRed;
//        if (gamepad1.bWasPressed()) isClose = !isClose;
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void stop() {}
//}