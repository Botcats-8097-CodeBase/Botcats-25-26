package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TurretTest", group = "Linear OpMode")
public class TurretTest extends LinearOpMode {

    double motorSpeed;
    private DcMotor turretMotor = null;
    private Servo angleServ = null;
    double servoPos = 0;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotor.class, "Zero");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        angleServ = hardwareMap.get(Servo.class, "serv");


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.3) {
                motorSpeed = gamepad1.right_trigger;
                turretMotor.setPower(motorSpeed);
            }

            if (gamepad1.a) {
                angleServ.setPosition(servoPos+=.1);
            }
            if (gamepad1.y) {
                angleServ.setPosition(servoPos-=.1);
            }

//            telemetry.addData("servo position", angleServ.getPosition());
        }
    }
}
