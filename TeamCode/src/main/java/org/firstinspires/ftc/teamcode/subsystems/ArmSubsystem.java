package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.MathUtils;

import static org.firstinspires.ftc.teamcode.config.ArmConfig.*;


@Config
public class ArmSubsystem extends SubsystemBase {
    private final GamepadEx gamepad;

    private final Motor motor;
    private final PIDController controller;

    private double position = 0.0d;
    private double offset = 0.0d;

    public static double SPEED = 25.0d;
    public static double ADJUSTMENT_SPEED = 10.0d;
    public static double MAX_POS = 900d;
    public static double SLOWDOWN_THRESHOLD = 100d;
    public static double SLOW_SPEED = 10.0d;

    public ArmSubsystem(GamepadEx gamepad, HardwareMap hardwareMap) {
        this.gamepad = gamepad;

        this.motor = new Motor(hardwareMap, "armMotor");
        this.motor.setInverted(true);
        this.motor.setRunMode(Motor.RunMode.RawPower);
        this.motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.motor.resetEncoder();

        this.controller = new PIDController(P, I, D);
    }

    @Override
    public void periodic() {
        this.controller.setTolerance(TOLERANCE);
        this.controller.setPID(P, I, D);

        offset += (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) * ADJUSTMENT_SPEED;

        double stickPos = gamepad.getLeftY() * Math.abs(gamepad.getLeftY());
        double slowSpeed = this.position / SLOWDOWN_THRESHOLD;
        double position = this.position + (this.position < SLOWDOWN_THRESHOLD && stickPos < 1d ? slowSpeed : 1d) * SPEED * stickPos;
        this.position = MathUtils.clamp(position, 0d, MAX_POS);

        double currentPosition = this.motor.getCurrentPosition() + offset;

        this.controller.setSetPoint(this.position);
        double output = this.controller.calculate(currentPosition);

        if(!this.controller.atSetPoint()) {
            this.motor.set(output);
        } else {
            this.motor.set(0);
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetPosition", this.position);
        packet.put("currentPosition", currentPosition);
        packet.put("power", output);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
    }
}
