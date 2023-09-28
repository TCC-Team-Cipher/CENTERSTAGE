package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SoftwareServo {
    private Servo servo;
    private double min;
    private double max;
    public double position;

    public SoftwareServo(HardwareMap hardwareMap, String name, double minValue, double maxValue) {
        servo  = hardwareMap.get(Servo.class, name);
        min = minValue;
        max = maxValue;
        position = min;
    }

    public void drive(double delta, double stickPosition) {
        position = Math.max(Math.min(position + stickPosition * delta, max), min);
        servo.setPosition(position);
    }
}
