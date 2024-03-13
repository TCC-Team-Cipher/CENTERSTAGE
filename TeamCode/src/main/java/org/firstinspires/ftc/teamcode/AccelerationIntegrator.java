package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class AccelerationIntegrator implements BNO055IMU.AccelerationIntegrator {
    private Position position;
    private Velocity velocity;
    private Acceleration acceleration;

    private long lastTime;

    @Override
    public void initialize(@NonNull BNO055IMU.Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity) {
        position = initialPosition;
        velocity = initialVelocity;
    }

    @Override
    public Position getPosition() {
        return position;
    }

    @Override
    public Velocity getVelocity() {
        return velocity;
    }

    @Override
    public Acceleration getAcceleration() {
        return acceleration;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        acceleration = linearAcceleration;

        long delta = acceleration.acquisitionTime - lastTime;
        lastTime = acceleration.acquisitionTime;
        double multiplier = delta / 1_000_000_000d;

        velocity.xVeloc += acceleration.xAccel * multiplier;
        velocity.yVeloc += acceleration.yAccel * multiplier;
        velocity.zVeloc += acceleration.zAccel * multiplier;

        position.x += linearAcceleration.xAccel * multiplier;
        position.y += linearAcceleration.yAccel * multiplier;
        position.z += linearAcceleration.zAccel * multiplier;
    }
}
