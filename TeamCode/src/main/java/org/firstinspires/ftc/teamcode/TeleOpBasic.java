package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.BasicRobot;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@TeleOp(name = "teleopbasic")
public class TeleOpBasic extends OpMode {

    List<LynxModule> allHubs;

    BasicRobot robot = new BasicRobot();

    public IMU imu;
    public double finalAngle;

    ElapsedTime et = new ElapsedTime();

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    @Override
    public void init() {
        initRobot();
    }

    @Override
    public void start() {
        super.start();
        et.reset();
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
        }

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw", yaw);
        telemetry.addData("Expected Yaw", finalAngle);

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
                        .rotateVector(-Math.toRadians(yaw))
                , gamepad1.right_stick_x, coefficient);

        telemetry.update();

    }

    void initRobot() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot.init(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        imu.resetYaw();
    }
}