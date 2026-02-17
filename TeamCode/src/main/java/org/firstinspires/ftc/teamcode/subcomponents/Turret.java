package org.firstinspires.ftc.teamcode.subcomponents;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.utils.TylerMath.normalize180;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotConstants;
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
    public Servo blockerServo;
    public CustomColorSensor lowColor = new CustomColorSensor();
    public CustomColorSensor highColor = new CustomColorSensor();

    public double distError = 0;

    ElapsedTime et = new ElapsedTime();
    ElapsedTime yawTimer = new ElapsedTime();

    double shootStartTimeMs = -1;
    boolean isShooting = false;
    double[] targetPreset = {-1, -1};
    double[] currentTargets = {0, 0};
    public double[] presetOffset = {0, 0};
    boolean isRed = false;
    boolean useDistError = true;

    boolean isSpinningUp = false;

    double[] robotPos = {0, 0, 0};

    public JoinedTelemetry pTelemetry;

    public Turret() {}

    public void updatePose(double[] pose) {
        robotPos = pose;
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

        if ((spinnerMotor1.isAtTargetVelocity() || shootStartTimeMs != -1) && isShooting) {
            //this code is made when we want a shoot sequence (we don't need this right now)
            if (shootStartTimeMs == -1) {
                shootStartTimeMs = et.milliseconds();
                intakeMotor.setPower(0);
            } else {
                double currTime = et.milliseconds() - shootStartTimeMs;

                if (currTime < 300) {
                    blockerServo.setPosition(RobotConstants.blockerShootingPos);
                } else if (currTime < 500) {
                    clutchServo.setPosition(RobotConstants.clutchEndPos);
                } else if (currTime < 1000) {
                    intakeMotor.setPower(RobotConstants.intakeMotorPower);
                } else if (currTime < 1250) {
                    intakeMotor.setPower(RobotConstants.intakeMotorPower + 0.3);
                }
            }
        } else {
            blockerServo.setPosition(RobotConstants.blockerBlockingPos);
        }

        double[] basePreset = (robotPos[0] > 40) ? RobotConstants.fullSpeedPreset.clone() : RobotConstants.closestSpeedPreset.clone();
        if (isShooting) {
            double[] goal = goalPos();
            double baseDist = 77;
            distError = Math.sqrt(Math.pow(goal[0] - robotPos[0], 2) + Math.pow(goal[1] - robotPos[1], 2)) - baseDist;
            boolean isShootClose = robotPos[0] < 40;

            if (targetPreset[0] == -1) {
                double shootSpeed = basePreset[0];

                if (isShootClose && useDistError) {
                    double kS = 0.006;
                    shootSpeed += distError * kS;
                }

                shootSpeed += presetOffset[0];


                spinnerMotor1.setTargetVelocity(shootSpeed);

                currentTargets[0] = shootSpeed;
            } else {
                pitchTurretServo.setPosition(targetPreset[0]);
            }

            if (targetPreset[1] == -1) {
                if (!spinnerMotor1.velocityFilter.isDataless()) {
                    double pos = basePreset[1];

                    if (isShootClose && useDistError) {
                        double kP = 0.005;
                        pos += distError * kP;
                    }

                    double error = spinnerMotor1.getVelocity() - spinnerMotor1.getTargetVelocity();

                    double kv = isShootClose ? 0.1 : 0.4; // increased far kv from 0.3
                    pos += error * kv;
                    pos = clip(pos, 0.0, 1.0);

                    pos += presetOffset[1];

                    pitchTurretServo.setPosition(pos);

                    currentTargets[1] = pos;
                }
            } else {
                pitchTurretServo.setPosition(targetPreset[1]);
            }

            teleData("shooting", true);
        } else if (isSpinningUp) {
            spinnerMotor1.setTargetVelocity(basePreset[0]);
            pitchTurretServo.setPosition(basePreset[1]);

            teleData("spinning up", true);
        } else {
            teleData("stopping", true);
        }

    }

    boolean isFirstTrigger = true;
    public void reverseIntake() {
        if (!isShooting) {
            if (isFirstTrigger) {
                isFirstTrigger = false;
            }
            isFirstStop = true;

            clutchServo.setPosition(RobotConstants.clutchEndPos);
            intakeMotor.setPower(-RobotConstants.intakeMotorPower);
        }
    }

    public void triggerIntake() {
        if (!isShooting) {
            isFirstStop = true;
            if (isFirstTrigger) {
                isFirstTrigger = false;
                clutchTimer.reset();
            }


            if (highColor.checkColor() || lowColor.checkColor()) {
                clutchServo.setPosition(RobotConstants.clutchStartPos);
            }

            if (clutchTimer.milliseconds() > 300) {
                intakeMotor.setPower(RobotConstants.intakeMotorPower);
            }
        }
    }

    boolean isFirstStop;
    public void stopIntake() {
        if (!isShooting) {
            if (isFirstStop) {
                isFirstStop = false;
                clutchTimer.reset();
            }
            intakeMotor.setPower(0);
            isFirstTrigger = true;

            if (clutchTimer.milliseconds() > 200) {
                if (!lowColor.checkColor() && !highColor.checkColor())
                    clutchServo.setPosition(RobotConstants.clutchEndPos);
            }
        }
    }


    private final ElapsedTime clutchTimer = new ElapsedTime();
    private int overrideState = 0;

    public void manualOverride() {
        switch (overrideState) {
            case 0:
                clutchServo.setPosition(RobotConstants.clutchEndPos);
                clutchTimer.reset();
                overrideState = 1;
                break;

            case 1:
                if (clutchTimer.milliseconds() >= 200) {
                    intakeMotor.setPower(RobotConstants.intakeMotorPower);
                    overrideState = 2;
                }
                break;

            case 2:
                intakeMotor.setPower(RobotConstants.intakeMotorPower);
                break;
        }
    }

    public void stopManualOverride() {
        overrideState = 0;
    }

    public void faceTo(double yaw) {
        yaw = TylerMath.wrap(yaw, 0, 360);
        if (yaw > 270) yaw -= 360;
        yaw = clip(yaw, RobotConstants.yawTurretMinAngle, RobotConstants.yawTurretMaxAngle);

        yawMotor.setTargetPosition(yaw);
    }

    public void spinUp() {
        isSpinningUp = true;
    }

    public double[] getCurrentTargets() {
        return currentTargets;
    }

    public void continueShootSequence() {
        if (!isShooting) {
            isShooting = true;
            isSpinningUp = false;
            shootStartTimeMs = -1;
        }
    }

    public double[] goalPos() {
        return isRed ? new double[]{-72, 68} : new double[]{-72, -68};
    }

    public void setShootPreset(double[] preset) {
        this.targetPreset = preset;
    }

    public void stopShootSequence() {
        isShooting = false;
        isSpinningUp = false;
        stopSpinner();
    }

    public void setGoalColor(boolean isRed) {
        this.isRed = isRed;
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

    public void useDistError(boolean useDistError) {
        this.useDistError = useDistError;
    }

    public double autoFace() {
        double[] g = goalPos();

        return TylerMath.wrap(-Math.toDegrees(Math.atan2(g[1] - robotPos[1], g[0] - robotPos[0])) + robotPos[2] + 180, -180, 180);
    }
    void teleData(String caption, Object data) {
        if (pTelemetry != null) pTelemetry.addData(caption, data);
    }

    public void init(HardwareMap hardwareMap) {
        yawMotor.init(hardwareMap, RobotConstants.yawTurretMotorName);
        yawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        yawMotor.setTargetPosition(0);
        yawMotor.setPosController(new PIDController(0.015, 0.0002, 0, 100));
        yawMotor.setMaxPower(0.4);

        spinnerMotor1.init(hardwareMap, RobotConstants.spinnerMotor1Name);
        spinnerMotor1.setDirection(RobotConstants.spinnerMotor1Direction);
        spinnerMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinnerMotor1.setMaxPower(1.1);
        spinnerMotor1.setMaxBoundsForTarget(0.01);
        spinnerMotor1.setVelController(new PIDFController(1.4, 0.002, 1.1, 0.420, 150));

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

        lowColor.init(hardwareMap, RobotConstants.lowColorSensorName);
        highColor.init(hardwareMap, RobotConstants.highColorSensorName);

        blockerServo = hardwareMap.get(Servo.class, RobotConstants.blockerServoName);

        et.reset();
        yawTimer.reset();
    }
}
