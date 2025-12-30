package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Configurable
public class RobotConstants {
    public static String leftBackDiveName = "leftBack";
    public static String leftFrontDiveName = "rightBack";
    public static String rightBackDiveName = "leftFront";
    public static String rightFrontDiveName = "rightFront";
    public static DcMotorSimple.Direction leftBackDiveDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftFrontDiveDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightBackDiveDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightFrontDiveDirection = DcMotorSimple.Direction.FORWARD;


    public static String intakeMotorName = "intake";
    public static DcMotorSimple.Direction intakeMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static double intakeMotorPower = 0.7;

    public static String yawTurretMotorName = "yawTurret";
    public static double yawTurretStartAngle = 332;
    public static double yawTurretMaxAngle = 80;
    public static double yawTurretMinAngle = -40;

    public static String yawTurretEncoderName = "yawTurretEncoder";

    public static String spinnerMotor1Name = "spinner1";
    public static DcMotorSimple.Direction spinnerMotor1Direction = DcMotorSimple.Direction.REVERSE;
    public static String spinnerMotor2Name = "spinner2";
    public static DcMotorSimple.Direction spinnerMotor2Direction = DcMotorSimple.Direction.REVERSE;

    public static String pitchTurretServoName = "pitchTurret";

    public static String pusherServoName = "pusher";
    public static double pusherStartPos = 0.7;
    public static double pusherEndPos = 0.0;


    // All presets are in this format {spinnerSpeed, pitchTurretPosition}
    public static double[] fullSpeedPreset = {2.1, 0.30};
    public static double[] autoSpeedPreset = {1.9, 0.30};
    public static double[] closestSpeedPreset = {1.50, 0.82};

}
