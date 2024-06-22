package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.GripPitch;

public class GripSubsystem extends SubsystemBase {
    public final GripPitch pitch;

    public final Claw left;
    public final Claw right;

    private final GamepadEx gamepad;

    public GripSubsystem(GamepadEx gamepad, HardwareMap hardwareMap, Telemetry telemetry) {
        this.gamepad = gamepad;

        this.pitch = new GripPitch(hardwareMap);

        left = Claw.left(hardwareMap);
        right = Claw.right(hardwareMap);
    }

    @Override
    public void periodic() {
        this.pitch.move(gamepad.getRightY() / 10d);

        left.set();
        right.set();
    }
}
