package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.BasicRobot;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@TeleOp(name = "mecanum drive test")
public class MecanumDriveTest extends OpMode {

    BasicRobot robot = new BasicRobot();

    ElapsedTime et = new ElapsedTime();

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
//                        .rotateVector(-Math.toRadians(yaw))
                , gamepad1.right_stick_x, coefficient);

        telemetry.update();

    }

    void initRobot() {
        robot.init(hardwareMap);
    }
}