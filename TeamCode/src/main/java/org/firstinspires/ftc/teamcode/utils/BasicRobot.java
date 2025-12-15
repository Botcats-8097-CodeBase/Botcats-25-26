package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class BasicRobot {
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;

    public BasicRobot() {}

    public void init(HardwareMap hardwareMap) {
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
    }

    public void drivePower(Vector2D moveVector, double rot, double speed) {
        double x = moveVector.x;
        double y = moveVector.y;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1.1);
        double frontLeftPower = (y + x + rot) / denominator * speed;
        double backLeftPower = (y - x + rot) / denominator * speed;
        double frontRightPower = (y - x - rot) / denominator * speed;
        double backRightPower = (y + x - rot) / denominator * speed;

        backRightDrive.setPower(backRightPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        frontLeftDrive.setPower(frontLeftPower);
    }
}
