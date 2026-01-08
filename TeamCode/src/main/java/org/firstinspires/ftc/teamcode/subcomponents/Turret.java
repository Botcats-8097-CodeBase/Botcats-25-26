package org.firstinspires.ftc.teamcode.subcomponents;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.utils.TylerMath.normalize180;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.NoiseFilter;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.RunToMotor;
import org.firstinspires.ftc.teamcode.utils.TylerMath;
import org.firstinspires.ftc.teamcode.utils.VelocityMotor;

public class Turret {
    public RunToMotor yawMotor = new RunToMotor();
    public AS5600 yawTurretEncoder;
    public VelocityMotor spinnerMotor1 = new VelocityMotor();
    public DcMotor spinnerMotor2;
    public Servo pitchTurretServo;
    public Servo clutchServo;
    public DcMotor intakeMotor;
    public ColorSensor lowColorSensor;

    ElapsedTime et = new ElapsedTime();
    ElapsedTime yawTimer = new ElapsedTime();

    double shootStartTimeMs = -1;
    boolean isShooting = false;
    double[] targetPreset = {0, 0};

    public Turret() {}

    public void init(HardwareMap hardwareMap) {
        yawMotor.init(hardwareMap, RobotConstants.yawTurretMotorName);
        yawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        yawMotor.setTargetPosition(0);
        yawMotor.setPosController(new PIDController(0.015, 0.0002, 0, 100));
        yawMotor.setMaxPower(0.4);

        spinnerMotor1.init(hardwareMap, RobotConstants.spinnerMotor1Name);
        spinnerMotor1.setDirection(RobotConstants.spinnerMotor1Direction);
        spinnerMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinnerMotor1.setMaxPower(1);
        spinnerMotor1.setMaxBoundsForTarget(0.01);
        spinnerMotor1.setVelController(new PIDFController(1.4, 0.002, 0, 0.420, 100));

        spinnerMotor2 = hardwareMap.get(DcMotor.class, RobotConstants.spinnerMotor2Name);
        spinnerMotor2.setDirection(RobotConstants.spinnerMotor2Direction);
        spinnerMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinnerMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerMotor2.setPower(0);

        pitchTurretServo = hardwareMap.get(Servo.class, RobotConstants.pitchTurretServoName);
        pitchTurretServo.setPosition(0.5);

        clutchServo = hardwareMap.get(Servo.class, RobotConstants.clutchServoName);
        clutchServo.setPosition(RobotConstants.clutchStartPos); //this should be in the "locked-disengaged" position

        yawTurretEncoder = hardwareMap.get(AS5600.class, RobotConstants.yawTurretEncoderName);

        intakeMotor = hardwareMap.get(DcMotor.class, RobotConstants.intakeMotorName);
        intakeMotor.setDirection(RobotConstants.intakeMotorDirection);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lowColorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.lowColorSensorName);
        lowColorSensor.enableLed(true);

        et.reset();
        yawTimer.reset();
    }

    public void loop() {
        double dt = yawTimer.seconds();
        yawTimer.reset();

        //TODO: Make this better in the future
        double inputYaw = yawTurretEncoder.getAngle0to360();
        if (inputYaw > 270) {
            inputYaw -= 360;
        }
        yawMotor.update(inputYaw);

        spinnerMotor1.update();
        spinnerMotor2.setPower(spinnerMotor1.getPower());

        if (spinnerMotor1.isAtTargetVelocity() && isShooting) {
            //this code is made when we want a shoot sequence (we don't need this right now)
            if (shootStartTimeMs == -1) {
                shootStartTimeMs = et.milliseconds();
                intakeMotor.setPower(0);
            } else {
                double currTime = et.milliseconds() - shootStartTimeMs;
                if (currTime < 500) {
                    clutchServo.setPosition(RobotConstants.clutchEndPos);
                } else if (currTime < 1000) {
                    intakeMotor.setPower(RobotConstants.intakeMotorPower);
                }
            }
        }
    }

    boolean isFirstTrigger = true;
    public void reverseIntake() {
        if (!isShooting) {
            if (isFirstTrigger) {
                clutchServo.setPosition(RobotConstants.clutchEndPos);
                isFirstTrigger = false;
            }
            intakeMotor.setPower(-RobotConstants.intakeMotorPower);
        }
    }

    public void triggerIntake() {
        if (!isShooting) {
            if (isFirstTrigger) {
                clutchServo.setPosition(RobotConstants.clutchEndPos);
                isFirstTrigger = false;
            }
            // 225 purple 157 green
            float[] hsv = new float[3];
            Color.RGBToHSV(lowColorSensor.red(), lowColorSensor.green(), lowColorSensor.blue(), hsv);
            if (Math.abs(hsv[0] - 225) < 10 || Math.abs(hsv[0] - 157) < 10) clutchServo.setPosition(RobotConstants.clutchStartPos);
            intakeMotor.setPower(RobotConstants.intakeMotorPower);
        }
    }

    public void manualOverride() {
        clutchServo.setPosition(RobotConstants.clutchEndPos);

        intakeMotor.setPower(RobotConstants.intakeMotorPower);
    }

    public void stopIntake() {
        if (!isShooting) {
            intakeMotor.setPower(0);
            isFirstTrigger = true;
        }
    }

    public void faceTo(double yaw) {
        yaw = TylerMath.wrap(yaw, 0, 360);
        if (yaw > 270) yaw -= 360;
        yaw = clip(yaw, RobotConstants.yawTurretMinAngle, RobotConstants.yawTurretMaxAngle);

        yawMotor.setTargetPosition(yaw);
    }

    public void goToPreset(double[] preset) {
        spinnerMotor1.setTargetVelocity(preset[0]);
        pitchTurretServo.setPosition(preset[1]);
    }

    public void goToPreset() {
        spinnerMotor1.setTargetVelocity(targetPreset[0]);
        pitchTurretServo.setPosition(targetPreset[1]);
    }

    public void continueShootSequence(double[] preset) {
        if (!isShooting) {
            isShooting = true;
            shootStartTimeMs = -1;
            goToPreset(preset);
        }
    }

    // gives spinner speed, and pitch turret position that varies depending on the current velocity
    // input turret.spinnerMotor1.getVelocity()) and preset
    public double[] varPreset(double currVel, double[] givenPreset) {
        double k = 0.5;
        double newPos = givenPreset[1] + currVel * k;

        givenPreset[1] = newPos;
        return givenPreset;
    }

    public double autoFace(double x, double y, double yaw, boolean isRed) {
        double gx;
        double gy;
        if (isRed) {
            gx = -72;
            gy = 72;
        } else {
            gx = -72;
            gy = -72;
        }

        double out =  TylerMath.wrap(-Math.toDegrees(Math.atan2(gy - y, gx - x)) + yaw + 180, -180, 180);

        return out;
    }

    public void setShootPreset(double[] preset) {
        this.targetPreset = preset;
    }

    public void stopShootSequence() {
        isShooting = false;
        stopSpinner();
    }

    public boolean currentlyShooting() {
        return isShooting;
    }

    public void stopSpinner() {
        spinnerMotor1.stop();
    }

    public double getCurrentFacing() {
        return yawMotor.getCurrentPosition();
    }

    private Double lastAngle0to360 = null;
    private double continuousAngleDeg = 0;
    public double getContinuousTurretAngle() {
        double current = yawTurretEncoder.getAngle180to180();

        if (lastAngle0to360 != null) {
            double delta = current - lastAngle0to360;

            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;

            continuousAngleDeg += delta;
        }

        lastAngle0to360 = current;
        return continuousAngleDeg;
    }

    private double shortestAngleDiff(double target, double current) {
        double diff = target - current;
        diff = normalize180(diff);
        return diff;
    }
}
