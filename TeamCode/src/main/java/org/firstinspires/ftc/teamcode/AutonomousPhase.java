package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Autonomous Phase", group="CENTERSTAGE")
public class AutonomousPhase extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
    }

    @Override
    public void loop() {
        telemetry.addData("Acceleration", robot.imu.getAcceleration());
        telemetry.addData("Position", robot.imu.getPosition());
    }
}
