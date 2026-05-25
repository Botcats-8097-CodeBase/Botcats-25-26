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
    public static double intakeMotorPower = 0.85;

    public static String lowColorSensorName = "lowColor";
    public static String highColorSensorName = "highColor";

    public static String yawTurretMotorName = "yawTurret";
    public static double yawTurretStartAngle = 86.57;
    public static double yawTurretMaxAngle = 180;
    public static double yawTurretMinAngle = -40;

    public static String yawTurretEncoderName = "yawTurretEncoder";

    public static String spinnerMotor1Name = "spinner1";
    public static DcMotorSimple.Direction spinnerMotor1Direction = DcMotorSimple.Direction.REVERSE;
    public static String spinnerMotor2Name = "spinner2";
    public static DcMotorSimple.Direction spinnerMotor2Direction = DcMotorSimple.Direction.REVERSE;

    public static String pitchTurretServoName = "pitchTurret";

    public static String clutchServoName = "clutch";
    public static double clutchStartPos = 0.485; //this should be in the "locked-disengaged" position
    // if we want free spin but disengaged, value = 0.67
    // if we want locked spin and disengaged, value = 0.68
    public static double clutchEndPos = 0.31;

    public static String hookServoName = "hook";
    public static double hookDownPos = 0;
    public static double hookUpPos = 0.125;

    public static String blockerServoName = "blocker";
    public static double blockerBlockingPos = 0.055;
    public static double blockerShootingPos = 0.348;

    public static String vectorServoName = "vector";
    public static double vectorInitPos = 0.3;
    public static double vectorDropPos = 0.0;

    // All presets are in this format {spinnerSpeed, pitchTurretPosition}
    public static double[] fullSpeedPreset = {2.20, 0.50}; // 80
    public static double[] closestSpeedPreset = {1.7, 0.10}; //{1.45, 0.50};
    //0.67

}
