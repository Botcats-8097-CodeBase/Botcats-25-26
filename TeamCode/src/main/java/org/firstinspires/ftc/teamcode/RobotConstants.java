package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Configurable
public class RobotConstants {
    public static String leftBackDiveName = "leftBack";
    public static String leftFrontDiveName = "leftFront";
    public static String rightBackDiveName = "rightBack";
    public static String rightFrontDiveName = "rightFront";
    public static DcMotorSimple.Direction leftBackDiveDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftFrontDiveDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightBackDiveDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightFrontDiveDirection = DcMotorSimple.Direction.FORWARD;


    public static String intakeMotorName = "intake";
    public static DcMotorSimple.Direction intakeMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static double intakeMotorPower = 0.75;

    public static String lowColorSensorName = "lowColor";
    public static String highColorSensorName = "highColor";

    public static String yawTurretMotorName = "yawTurret";
    public static double yawTurretStartAngle = 64.4;
    public static double yawTurretMaxAngle = 80;
    public static double yawTurretMinAngle = -40;

    public static String yawTurretEncoderName = "yawTurretEncoder";

    public static String spinnerMotor1Name = "spinner1";
    public static DcMotorSimple.Direction spinnerMotor1Direction = DcMotorSimple.Direction.REVERSE;
    public static String spinnerMotor2Name = "spinner2";
    public static DcMotorSimple.Direction spinnerMotor2Direction = DcMotorSimple.Direction.REVERSE;

    public static String pitchTurretServoName = "pitchTurret";

    public static String clutchServoName = "clutch";
    public static double clutchStartPos = 0.68; //this should be in the "locked-disengaged" position
    // if we want free spin but disengaged, value = 0.67
    // if we want locked spin and disengaged, value = 0.68
    public static double clutchEndPos = 0.61;
    public static String pusherServoName = "pusher";
    public static double pusherStartPos = 0.7;
    public static double pusherEndPos = 0.0;


    // All presets are in this format {spinnerSpeed, pitchTurretPosition}
    public static double[] fullSpeedPreset = {2.3, 0.10};
    public static double[] autoSpeedPreset = {1.50, 0.70};
    public static double[] closestSpeedPreset = {1.57, 0.70};

}
