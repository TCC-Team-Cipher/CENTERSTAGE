package org.firstinspires.ftc.teamcode.op_modes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.GripCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;

@TeleOp(name = "Driver Controlled", group = "CENTERSTAGE")
public class DriverControlled extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx driveGamepad = new GamepadEx(gamepad1);
        GamepadEx armGamepad = new GamepadEx(gamepad2);

        DriveSubsystem driveSubsystem = new DriveSubsystem(driveGamepad, hardwareMap);
        ArmSubsystem armSubsystem = new ArmSubsystem(armGamepad, hardwareMap);
        GripSubsystem clawSubsystem = new GripSubsystem(armGamepad, hardwareMap, telemetry);
        TelemetrySubsystem telemetrySubsystem = new TelemetrySubsystem(telemetry);

        GripCommand gripCommand = new GripCommand(clawSubsystem);

        Button leftGripToggle = new GamepadButton(
                armGamepad, GamepadKeys.Button.LEFT_BUMPER
        );
        leftGripToggle.whenPressed(gripCommand::toggleLeft);

        Button rightGripToggle = new GamepadButton(
                armGamepad, GamepadKeys.Button.RIGHT_BUMPER
        );
        rightGripToggle.whenPressed(gripCommand::toggleRight);

        Button top = new GamepadButton(
                armGamepad, GamepadKeys.Button.Y
        );
        Button backboard = new GamepadButton(
                armGamepad, GamepadKeys.Button.B
        );
        Button bottom = new GamepadButton(
                armGamepad, GamepadKeys.Button.A
        );
        top.whenPressed(gripCommand::top);
        backboard.whenPressed(gripCommand::backboard);
        bottom.whenPressed(gripCommand::bottom);

        register(driveSubsystem, armSubsystem, clawSubsystem, telemetrySubsystem);
        gripCommand.schedule();
    }
}