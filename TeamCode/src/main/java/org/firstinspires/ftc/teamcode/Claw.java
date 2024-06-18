package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Claw {

    private static final double MIN_POSITION = 0d;
    private static final double MAX_POSITION = 1d;

    private final ServoEx claw;
    private boolean open;

    public Claw(HardwareMap hardwareMap, String name) {
        claw = new SimpleServo(hardwareMap, name, 0.0d, 0.0d);
        this.open = false;
    }

    public void toggle() {
        this.set(!this.open);
    }

    public void set(boolean open) {
        this.open = open;
        this.claw.setPosition(this.open ? MAX_POSITION : MIN_POSITION);
    }
}
