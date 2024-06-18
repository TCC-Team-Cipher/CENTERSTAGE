package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.old.Arm;
import org.firstinspires.ftc.teamcode.old.Grip;
import org.firstinspires.ftc.teamcode.old.MecanumDrive;

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
