package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

public class MecanumDrive {
    private final Telemetry telemetry;

    private final DcMotorEx motor0;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final DcMotorEx motor3;

    public MecanumDrive(OpMode opMode) {
        telemetry = opMode.telemetry;

        motor0 = opMode.hardwareMap.get(DcMotorEx.class, "motor0");
        motor1 = opMode.hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = opMode.hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = opMode.hardwareMap.get(DcMotorEx.class, "motor3");

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor0.setDirection(DcMotorEx.Direction.FORWARD);
        motor1.setDirection(DcMotorEx.Direction.REVERSE);
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor3.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void drive(double dir, double turn, double power) {
        double power03 = Math.sin(dir - 0.25 * Math.PI) * power;
        double power12 = Math.sin(dir + 0.25 * Math.PI) * power;

        double[] powers = {
                power03 - turn,
                power12 + turn,
                power12 - turn,
                power03 + turn
        };

        double[] scaled = Arrays.stream(powers).map(element -> element * 360).toArray();

        motor0.setVelocity(scaled[0], AngleUnit.DEGREES);
        motor1.setVelocity(scaled[1], AngleUnit.DEGREES);
        motor2.setVelocity(scaled[2], AngleUnit.DEGREES);
        motor3.setVelocity(scaled[3], AngleUnit.DEGREES);

        telemetry.addData("Mecanum Drive", "dir: (%.2f), turn: (%.2f), power: (%.2f)", dir, turn, power);
        telemetry.addData("Mecanum Drive", "motor0: (%.2f), motor1: (%.2f), motor2: (%.2f), motor3: (%.2f)", scaled[0], scaled[1], scaled[2], scaled[3]);
    }
}
