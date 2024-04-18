package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Robot {

    public Arm arm;
    public Grip grip;

    public MecanumDrive mecanumDrive;

    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    public Robot(OpMode opMode) {
        grip = new Grip(opMode);
        arm = new Arm(opMode);
        mecanumDrive = new MecanumDrive(opMode);

        leftDistanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
    }

    public void start() {
        arm.start();
    }
}
