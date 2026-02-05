package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

public class OctoConstants {

    public int portX = 0;

    public int portY = 1;

    public int velocitySampleInterval = 50;

    public float imuScalar = 1f;

    /** The Y Offset of the Forward Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: 1 */
    public  float forwardPodY = 1;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: -2.5 */
    public  float strafePodX = -2.5f;

    /** The Unit of Distance that the Pinpoint uses to measure distance
     * Default Value: DistanceUnit.INCH */
    public  DistanceUnit distanceUnit = DistanceUnit.INCH;

    /** The name of the Pinpoint in the hardware map (name of the I2C port it is plugged into)
     * Default Value: "pinpoint" */
    public  String hardwareMapName = "pinpoint";

    /** The Encoder Direction for the Forward Encoder (Deadwheel)
     * Default Value: OctoQuad.EncoderDirection.REVERSED */
    public  OctoQuad.EncoderDirection forwardEncoderDirection = OctoQuad.EncoderDirection.REVERSE;

    /** The Encoder Direction for the Strafe Encoder (Deadwheel)
     * Default Value: OctoQuad.EncoderDirection.FORWARD */
    public  OctoQuad.EncoderDirection strafeEncoderDirection = OctoQuad.EncoderDirection.FORWARD;

    /**
     * This creates a new PinpointConstants with default values.
     */
    public OctoConstants() {
        defaults();
    }

    public OctoConstants portX(int port) {
        this.portX = port;
        return this;
    }

    public OctoConstants portY(int port) {
        this.portY = port;
        return this;
    }

    public OctoConstants imuScalar(float imuScalar) {
        this.imuScalar = imuScalar;
        return this;
    }


    public OctoConstants forwardPodY(float forwardPodY) {
        this.forwardPodY = forwardPodY;
        return this;
    }

    public OctoConstants strafePodX(float strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public OctoConstants distanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }

    public OctoConstants hardwareMapName(String hardwareMapName) {
        this.hardwareMapName = hardwareMapName;
        return this;
    }

    public OctoConstants forwardEncoderDirection(OctoQuad.EncoderDirection forwardEncoderDirection) {
        this.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public OctoConstants strafeEncoderDirection(OctoQuad.EncoderDirection strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public void defaults() {
        forwardPodY = 1;
        strafePodX = -2.5f;
        distanceUnit = DistanceUnit.INCH;
        hardwareMapName = "pinpoint";
        forwardEncoderDirection = OctoQuad.EncoderDirection.REVERSE;
        strafeEncoderDirection = OctoQuad.EncoderDirection.FORWARD;
    }
}
