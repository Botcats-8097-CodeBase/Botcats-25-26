/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name="customizableTester")
public class CustomizableTester extends LinearOpMode {

    public static double motorC0Power = 0.0;
    public static double motorC1Power = 0.0;
    public static double motorC2Power = 0.0;
    public static double motorC3Power = 0.0;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorC0 = null;
    private DcMotor motorC1 = null;
    private DcMotor motorC2 = null;
    private DcMotor motorC3 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorC0 = hardwareMap.get(DcMotor.class, "motorC0");
        motorC1 = hardwareMap.get(DcMotor.class, "motorC1");
        motorC2 = hardwareMap.get(DcMotor.class, "motorC2");
        motorC3 = hardwareMap.get(DcMotor.class, "motorC2");

        motorC0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorC1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorC2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorC3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorC0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorC1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorC2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorC3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorC0.setDirection(DcMotor.Direction.FORWARD);
        motorC1.setDirection(DcMotor.Direction.FORWARD);
        motorC2.setDirection(DcMotor.Direction.FORWARD);
        motorC3.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            motorC0.setPower(motorC0Power);

            motorC1.setPower(motorC1Power);

            motorC2.setPower(motorC2Power);

            motorC3.setPower(motorC3Power);
        }
    }
}
