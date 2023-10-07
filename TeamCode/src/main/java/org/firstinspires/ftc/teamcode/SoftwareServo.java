package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SoftwareServo {
    private Telemetry telemetry;
    private String name;
    private Servo servo;
    private double min;
    private double max;
    public double position;

    public SoftwareServo(OpMode opMode, String nameValue, double minValue, double maxValue) {
        telemetry = opMode.telemetry;
        name = nameValue;
        servo  = opMode.hardwareMap.get(Servo.class, name);
        min = minValue;
        max = maxValue;
        position = min;
    }

    public void drive(double delta, double stickPosition) {
        position = Math.max(Math.min(position + stickPosition * delta, max), min);
        servo.setPosition(position);

        telemetry.addData(name, "position: (%.2f)", position);
    }
}
