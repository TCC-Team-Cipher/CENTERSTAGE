package org.firstinspires.ftc.teamcode.op_modes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Blue Backboard Side", group = "CENTERSTAGE", preselectTeleOp = "Driver Controlled")
public class AutonomousBlueBackboard extends AutonomousPhase {
    public AutonomousBlueBackboard() {
        super(false, true);
    }
}
