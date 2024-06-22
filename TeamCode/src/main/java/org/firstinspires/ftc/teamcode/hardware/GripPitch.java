package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class GripPitch {
    public static double MIN_PITCH_POSITION = 0d;
    public static double MAX_PITCH_POSITION = .35d;

    private final ServoEx servo;
    private double position;

    public GripPitch(HardwareMap hardwareMap) {
        servo = new SimpleServo(hardwareMap, "pitch", 0d, 0d);
        position = MAX_PITCH_POSITION;
        servo.setPosition(position);
    }

    public  void set() {
        this.servo.setPosition(position);
    }

    public void set(double position) {
        this.position = position;
        set();
    }

    public void move(double difference) {
        this.position += difference;
        position = MathUtils.clamp(position, MIN_PITCH_POSITION, MAX_PITCH_POSITION);
        set();
    }
}
