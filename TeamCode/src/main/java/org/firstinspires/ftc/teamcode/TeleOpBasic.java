package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@TeleOp(name = "teleopbasic")
public class TeleOpBasic extends OpMode {

    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;

    public IMU imu;

    public double angle;
    public double finalAngle;

    ElapsedTime et = new ElapsedTime();

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
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

        if (gamepad1.y) {
            imu.resetYaw();
            imu.initialize(parameters);
        }

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw", yaw);
        telemetry.addData("Expected Yaw", finalAngle);

        angle = robotOrientation.getYaw(AngleUnit.RADIANS);

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        // rotates the left stick and changes the x & y values accordingly (field centric)
        Vector2D inputVector = new Vector2D(x, y);
        Vector2D rotatedVector = inputVector.rotateVector(-angle);

        x = rotatedVector.x;
        y = rotatedVector.y;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // pressurising the right trigger slows down the drive train
        double coefficient = 0.35;
        if(gamepad1.right_trigger < 0.5)
        {
            telemetry.addData("Speed Mode", "off");
        }
        else
        {
            telemetry.addData("Speed Mode", "trigger on");
            coefficient = 1;
        }

        backRightDrive.setPower(backRightPower * coefficient);
        backLeftDrive.setPower(backLeftPower * coefficient);
        frontRightDrive.setPower(frontRightPower * coefficient);
        frontLeftDrive.setPower(frontLeftPower * coefficient);







        telemetry.update();

    }

    void initRobot() {
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        backLeftDrive = hardwareMap.get(DcMotor.class, RobotConstants.leftBackDiveName);
        backRightDrive = hardwareMap.get(DcMotor.class, RobotConstants.rightBackDiveName);
        frontLeftDrive = hardwareMap.get(DcMotor.class, RobotConstants.leftFrontDiveName);
        frontRightDrive = hardwareMap.get(DcMotor.class, RobotConstants.rightFrontDiveName);

        backLeftDrive.setDirection(RobotConstants.leftBackDiveDirection);
        backRightDrive.setDirection(RobotConstants.rightBackDiveDirection);
        frontLeftDrive.setDirection(RobotConstants.leftFrontDiveDirection);
        frontRightDrive.setDirection(RobotConstants.rightFrontDiveDirection);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        imu.resetYaw();
    }
}