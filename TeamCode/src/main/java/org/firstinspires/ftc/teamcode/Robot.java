package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Robot {

    public Arm arm;
    public Grip grip;
    public MecanumDrive mecanumDrive;
    public BNO055IMU imu;

    public Robot(OpMode opMode) {
        grip = new Grip(opMode);
        mecanumDrive = new MecanumDrive(opMode);
        arm = new Arm(opMode);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.accelerationIntegrationAlgorithm = new AccelerationIntegrator();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(DistanceUnit.METER, .0d, 0.0d, 0.0d, 0), new Velocity(), 10);
    }

    public void start() {
        arm.start();
    }
}
