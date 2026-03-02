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
    public enum IntakeState {
        FORWARD,
        STOP,
        REVERSE
    }

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
    boolean wasShooting = true;
    double shootStopTimeMs = -1;
    public boolean isShooting = false;
    double[] targetPreset = {-1, -1};
    double[] currentTargets = {0, 0};
    public double[] presetOffset = {0, 0};
    boolean isRed = false;
    boolean useDistError = true;

    boolean isSpinningUp = false;
    IntakeState intakeState = IntakeState.STOP;

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

        boolean isShootClose = robotPos[0] < 40;
        double[] basePreset = isShootClose ? RobotConstants.closestSpeedPreset.clone() : RobotConstants.fullSpeedPreset.clone();

        if (targetPreset[0] == -1) {
            double shootSpeed = basePreset[0];

            if (isShootClose && useDistError) {
                double kS = 0.003;
                shootSpeed += distError * kS;
            }

            shootSpeed += presetOffset[0];
            shootSpeed = clip(shootSpeed, basePreset[0] - 0.3, basePreset[0] + 0.3);
            currentTargets[0] = shootSpeed;
            teleData("Check Running", true);
        } else {
            currentTargets[0] = targetPreset[0];
            teleData("Check Running", false);
        }

        if (targetPreset[1] == -1) {
            if (!spinnerMotor1.velocityFilter.isDataless()) {
                double pos = basePreset[1];

                if (isShootClose && useDistError) {
                    double kP = 0.003;
                    pos += distError * kP;
                }

                double error = spinnerMotor1.getVelocity() - spinnerMotor1.getTargetVelocity();

                double kv = isShootClose ? 0.1 : 0.4; // increased far kv from 0.3
                pos += error * kv;
                pos = clip(pos, 0.0, 1.0);
                pos = clip(pos, basePreset[1] - 0.3, basePreset[1] + 0.3);

                pos += presetOffset[1];

                currentTargets[1] = pos;
            }
        } else {
            currentTargets[1] = targetPreset[1];
        }

        if (isShooting) {
            wasShooting = true;

            double[] goal = goalPos();
            double baseDist = 77;
            distError = Math.sqrt(Math.pow(goal[0] - robotPos[0], 2) + Math.pow(goal[1] - robotPos[1], 2)) - baseDist;

            if (spinnerMotor1.isAtTargetVelocity() || shootStartTimeMs != -1) {
                //this code is made when we want a shoot sequence (we don't need this right now)
                if (shootStartTimeMs == -1) {
                    shootStartTimeMs = et.milliseconds();
                    intakeMotor.setPower(0);
                } else {
                    double currTime = et.milliseconds() - shootStartTimeMs;

                    if (currTime < 100) {
                        blockerServo.setPosition(RobotConstants.blockerShootingPos);
                    } else if (currTime < 300) {
                        clutchServo.setPosition(RobotConstants.clutchEndPos);
                    } else {
                        intakeMotor.setPower(RobotConstants.intakeMotorPower);
                    }
                }
            } else {
                intakeMotor.setPower(0);
                blockerServo.setPosition(RobotConstants.blockerBlockingPos);
            }

            spinnerMotor1.setTargetVelocity(currentTargets[0]);
            pitchTurretServo.setPosition(currentTargets[1]);
        } else {
            if (isSpinningUp) {
                spinnerMotor1.setTargetVelocity(currentTargets[0]);
                pitchTurretServo.setPosition(currentTargets[1]);
            } else {
                stopSpinner();
            }

            if (wasShooting) {
                shootStopTimeMs = et.milliseconds();
                wasShooting = false;
            } else {
                double currTime = et.milliseconds() - shootStopTimeMs;

                // todo inspect 1st case blocker situation (open when shooting)
                if (currTime > 800) {
                    blockerServo.setPosition(RobotConstants.blockerBlockingPos);
                }
            }


            if (intakeState != IntakeState.STOP) {
                if (isFirstIntake) {
                    clutchTimer.reset();
                    isFirstIntake = false;
                }

                if (highColor.checkColor() && lowColor.checkColor()) {
                    clutchServo.setPosition(RobotConstants.clutchStartPos);
                }

                if (clutchTimer.milliseconds() > 150) {
                    intakeMotor.setPower(((intakeState == IntakeState.FORWARD) ? 1 : -1) * RobotConstants.intakeMotorPower);
                }
            } else {
                if (!isFirstIntake) {
                    clutchTimer.reset();
                    isFirstIntake = true;
                }
                intakeMotor.setPower(0);

                if (clutchTimer.milliseconds() > 200) {
                    if (!lowColor.checkColor() || !highColor.checkColor())
                        clutchServo.setPosition(RobotConstants.clutchEndPos);
                }
            }
        }

        spinnerMotor1.update();
        spinnerMotor2.setPower(spinnerMotor1.getPower());

    }

    boolean isFirstIntake = true;
    public void reverseIntake() {
        intakeState = IntakeState.REVERSE;
    }

    public void triggerIntake() {
        intakeState = IntakeState.FORWARD;
    }

    public void stopIntake() {
        intakeState = IntakeState.STOP;
    }


    private final ElapsedTime clutchTimer = new ElapsedTime();

    public void manualOverride() {
        shootStartTimeMs = et.milliseconds();
        intakeMotor.setPower(0);
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

    public void spinUp(boolean isSpinningUp) {
        this.isSpinningUp = isSpinningUp;
    }

    public double[] getCurrentTargets() {
        return currentTargets;
    }

    public void continueShootSequence() {
        if (!isShooting) {
            isShooting = true;
            shootStartTimeMs = -1;
        }
    }

    public void stopShootSequence() {
        isShooting = false;
    }

    public double[] goalPos() {
        return isRed ? new double[]{-68, 72} : new double[]{-68, -72};
    }

    public void setShootPreset(double[] preset) {
        this.targetPreset = preset;
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

        return TylerMath.wrap(-Math.toDegrees(Math.atan2(g[1] - robotPos[1], g[0] - robotPos[0])) + robotPos[2], -180, 180);
    }
    void teleData(String caption, Object data) {
        if (pTelemetry != null) pTelemetry.addData(caption, data);
    }

    public void init(HardwareMap hardwareMap) {
        yawMotor.init(hardwareMap, RobotConstants.yawTurretMotorName);
        yawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        yawMotor.setTargetPosition(0);
        yawMotor.setPosController(new PIDController(0.015, 0.0002, 0, 100));
        yawMotor.setMaxPower(0);//0.4);

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
        blockerServo.setPosition(RobotConstants.blockerBlockingPos);

        et.reset();
        yawTimer.reset();
    }
}
