package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    private static final int SPEED = 580;
    private static final int MAX_POSITION = 1750;

    private final Telemetry telemetry;
    private final DcMotorEx motor;
    private final TouchSensor limitSwitch;

    private double position;

    public Arm(OpMode opMode) {
        telemetry = opMode.telemetry;
        motor = opMode.hardwareMap.get(DcMotorEx.class, "armMotor");
        limitSwitch = opMode.hardwareMap.get(TouchSensor.class, "limitSwitch");

        position = 0.0d;

        motor.setTargetPosition(0);

        home();
    }

    private void home() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!limitSwitch.isPressed()) {
            motor.setPower(-0.25);
        }
        while (limitSwitch.isPressed()) {
            motor.setPower(0.1);
        }
        while (!limitSwitch.isPressed()) {
            motor.setPower(-0.1);
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void start() {
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setPower(1.0d);
    }

    public void drive(double delta, double drive) {
        position += drive * delta * SPEED;
        position = Math.max(0.0d, Math.min(position, MAX_POSITION));

        motor.setTargetPosition((int)Math.round(this.position));

        telemetry.addData("Arm", "position: (%.2f)", position);
    }
}
