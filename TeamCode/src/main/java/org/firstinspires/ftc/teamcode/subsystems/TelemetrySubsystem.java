package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySubsystem extends SubsystemBase {
    private final Telemetry telemetry;

    public TelemetrySubsystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.update();
    }
}
