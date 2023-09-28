package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class MecanumDrive {
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;

    public MecanumDrive(HardwareMap hardwareMap) {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double dir, double power, double turn) {
        double power03 = Math.sin(dir - 0.25 * Math.PI) * power;
        double power12 = Math.sin(dir + 0.25 * Math.PI) * power;

        List<Double> powers = Arrays.asList(
                power03 - turn,
                power12 + turn,
                power12 - turn,
                power03 + turn
        );

        double biggest = powers.stream().reduce(1.0d, (subtotal, element) -> {
            double abs = Math.abs(element);
            if (abs > subtotal) {
                return abs;
            }
            return subtotal;
        });

        List<Double> scaled = powers.stream().map((element) -> element / biggest).collect(Collectors.toList());

        motor0.setPower(scaled.get(0));
        motor1.setPower(scaled.get(1));
        motor2.setPower(scaled.get(2));
        motor3.setPower(scaled.get(3));
    }
}
