package org.firstinspires.ftc.teamcode.op_modes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor Test")
@Disabled
public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor left = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor right = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        telemetry.addLine("Distance Sensors")
                .addData("left", () -> left.getDistance(DistanceUnit.CM))
                .addData("right", () -> right.getDistance(DistanceUnit.CM));

        while (!isStopRequested()) {
            telemetry.update();
        }
    }
}
