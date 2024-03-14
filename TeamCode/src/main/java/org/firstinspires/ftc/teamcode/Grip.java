package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grip {
    private final Servo pitchServo;
    private final CRServo yawServo;
    private double pitchPosition;

    private final Servo rightServo;
    boolean rightOpen;

    private final Servo leftServo;
    boolean leftOpen;

    private final Telemetry telemetry;

    public Grip(OpMode opMode) {
        pitchServo = opMode.hardwareMap.get(Servo.class, "pitch");
        yawServo = opMode.hardwareMap.get(CRServo.class, "yaw");
        pitchPosition = 1.0d;

        rightServo = opMode.hardwareMap.get(Servo.class, "right");
        rightOpen = false;

        leftServo = opMode.hardwareMap.get(Servo.class, "left");
        leftOpen = false;

        telemetry = opMode.telemetry;
    }

    public void drive(double pitch, double yaw) {
        pitchPosition += pitch;
        pitchPosition = Math.max(0.0d, Math.min(1.0d, pitchPosition));
        pitchServo.setPosition(pitchPosition);

        yawServo.setPower(yaw);

        telemetry.addData("Grip", "pitchPosition: (%.2f), yaw: (%.2f)", pitchPosition, yaw);
    }

    public void toggleRight() {
        rightOpen = !rightOpen;
        rightServo.setPosition(rightOpen ? 1.0d : 0.25d);
    }

    public void toggleLeft() {
        leftOpen = !leftOpen;
        leftServo.setPosition(leftOpen ? 1.0d : 0.25d);
    }
}
