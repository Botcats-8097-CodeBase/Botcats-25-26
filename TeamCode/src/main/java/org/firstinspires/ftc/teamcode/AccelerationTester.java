package org.firstinspires.ftc.teamcode;

import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@TeleOp(name = "acceleration tester")
public class AccelerationTester extends OpMode {

    List<LynxModule> allHubs;

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;

    public IMU imu;

    public double angle;
    public double finalAngle;

    GoBildaPinpointDriver pinpoint;

    ElapsedTime et = new ElapsedTime();

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    GraphManager graphManager = PanelsGraph.INSTANCE.getManager();

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

        double posX = pinpoint.getPosX(DistanceUnit.INCH);
        double posY = pinpoint.getPosY(DistanceUnit.INCH);
        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);

        panelsTelemetry.debug("posX", posX);
        panelsTelemetry.debug("posY", posY);
        panelsTelemetry.debug("velX", velX);
        panelsTelemetry.debug("velY", velY);

        graphManager.addData("posX", posX);
        graphManager.addData("posY", posY);
        graphManager.addData("velX", velX);
        graphManager.addData("velY", velY);



        panelsTelemetry.update();

        graphManager.update();





        telemetry.update();

    }

    void initRobot() {
        allHubs = hardwareMap.getAll(LynxModule.class);

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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Seet the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void configurePinpoint(){
        pinpoint.setOffsets(-3.125, 3.125, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }
}