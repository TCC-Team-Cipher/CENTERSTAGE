package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final GamepadEx gamepad;

    private final RevIMU imu;

    public DriveSubsystem(GamepadEx gamepad, HardwareMap hardwareMap) {
        this.gamepad = gamepad;

        Motor frontLeft = new Motor(hardwareMap, "leftFront");
        Motor backRight = new Motor(hardwareMap, "rightBack");
        Motor frontRight = new Motor(hardwareMap, "rightFront");
        Motor backLeft = new Motor(hardwareMap, "leftBack");

        drive = new MecanumDrive(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );

        this.imu = new RevIMU(hardwareMap, "imu");
        this.imu.init();
    }

    @Override
    public void periodic() {
        drive.driveRobotCentric(
                -gamepad.getLeftX(),
                -gamepad.getLeftY(),
                -gamepad.getRightX(),
                false
        );
    }
}
