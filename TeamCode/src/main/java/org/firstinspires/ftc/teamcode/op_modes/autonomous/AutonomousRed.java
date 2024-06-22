package org.firstinspires.ftc.teamcode.op_modes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Red Side", group = "CENTERSTAGE", preselectTeleOp = "Driver Controlled")
public class AutonomousRed extends AutonomousPhase {
    public AutonomousRed() {
        super(true, true);
    }
}
