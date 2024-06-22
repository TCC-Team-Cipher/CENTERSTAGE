package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;

@Config
public class Claw {

    public static double MIN_POSITION = 0.2d;
    public static double MAX_POSITION = 1d;

    private final ServoEx claw;
    private boolean open;
    private final boolean reverse;

    public Claw(HardwareMap hardwareMap, String name, boolean reverse) {
        this.claw = new SimpleServo(hardwareMap, name, 0.0d, 0.0d);
        this.reverse = reverse;
        this.set(false);
    }

    public void toggle() {
        this.set(!this.open);
    }

    public void set(boolean open) {
        this.open = open;
        this.set();
    }

    public void set() {
        double position = this.open ? MAX_POSITION : MIN_POSITION;
        this.claw.setPosition(this.reverse ? 1 - position : position);
    }

    public static Claw left(HardwareMap hardwareMap) {
        return new Claw(hardwareMap, "left", false);
    }

    public static Claw right(HardwareMap hardwareMap) {
        return new Claw(hardwareMap, "right", true);
    }
}
