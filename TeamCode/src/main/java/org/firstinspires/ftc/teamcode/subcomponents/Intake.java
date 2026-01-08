package org.firstinspires.ftc.teamcode.subcomponents;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.BasicRobot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

public class Intake {

    DcMotor intakeMotor;
    ColorSensor lowColorSensor;

    public Intake() {}

    public void init(HardwareMap hardwareMap) {

    }

    public void reverseTrigger() {
        intakeMotor.setPower(-RobotConstants.intakeMotorPower);
    }

    public void trigger() {
         intakeMotor.setPower(RobotConstants.intakeMotorPower);
    }

    public void stop() { intakeMotor.setPower(0); }

}