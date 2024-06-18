package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Claw;

public class GripSubsystem extends SubsystemBase {
    private static final double MIN_PITCH_POSITION = 0d;
    private static final double MAX_PITCH_POSITION = .33d;

    private final ServoEx pitch;
    private double pitchPosition;

    public final Claw left;
    public final Claw right;

    private final GamepadEx gamepad;


    public GripSubsystem(GamepadEx gamepad, HardwareMap hardwareMap, Telemetry telemetry) {
        this.gamepad = gamepad;

        pitch = new SimpleServo(hardwareMap, "pitch", 0d, 0d);
        pitchPosition = MIN_PITCH_POSITION;

        left = new Claw(hardwareMap, "left");
        right = new Claw(hardwareMap, "right");

        telemetry.addLine("Grip")
                .addData("Pitch", pitch::getPosition);
    }

    @Override
    public void periodic() {
        pitchPosition += gamepad.getLeftY() / 10d;
        pitchPosition = MathUtils.clamp(pitchPosition, MIN_PITCH_POSITION, MAX_PITCH_POSITION);
        pitch.setPosition(pitchPosition);
    }
}
