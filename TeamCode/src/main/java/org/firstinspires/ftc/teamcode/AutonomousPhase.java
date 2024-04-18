package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Optional;

@Autonomous(name = "Autonomous Phase", group = "CENTERSTAGE", preselectTeleOp = "DriverControlled")
public class AutonomousPhase extends LinearOpMode {
    private Robot robot;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final double FORWARDS = Math.PI * 1.5d;
    private static final double BACKWARDS = Math.PI * 0.5d;
    private static final double RIGHT = 0.0d;
    private static final double LEFT = Math.PI;

    enum Position {
        LEFT,
        RIGHT,
        MIDDLE,
    }

    private Position position;

    private void update() {
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        position = null;

        telemetry.addLine()
                .addData("left", () -> robot.leftDistanceSensor.getDistance(DistanceUnit.CM))
                .addData("right", () -> robot.rightDistanceSensor.getDistance(DistanceUnit.CM))
                .addData("position", () -> position != null ? position.toString() : "null");

        waitForStart();

        robot.start();

        runtime.reset();

        robot.arm.setPosition(200.0d);

        doUpdates(1.0d);

        moveToStart();

        findPosition();

        moveToPosition();

        dropPixel();

        doUpdates(10.0d);
    }

    private void stopMecanumDrive() {
        robot.mecanumDrive.drive(0.0d, 0.0d, 0.0d);
    }

    private void moveToStart() {
        double end = runtime.time() + 1.15d;
        while (runtime.time() < end) {
            robot.mecanumDrive.drive(FORWARDS, 0.0d, 3.0d);
            update();
        }
        stopMecanumDrive();
    }

    private void findPosition() {
        double end = runtime.time() + 1.0d;

        boolean leftFound = false;
        boolean rightFound = false;

        while (runtime.time() < end) {
            robot.mecanumDrive.drive(FORWARDS, 0.0d, 1.0d);

            if (!(leftFound || rightFound)) {
                leftFound = robot.leftDistanceSensor.getDistance(DistanceUnit.CM) <= 30.0d;
                rightFound = robot.rightDistanceSensor.getDistance(DistanceUnit.CM) <= 30.0d;
            }

            update();
        }

        if (leftFound) {
            position = Position.LEFT;
        } else if (rightFound) {
            position = Position.RIGHT;
        } else {
            position = Position.MIDDLE;
        }

        stopMecanumDrive();
    }

    private void moveToPosition() {
        Optional<Double> turn = Optional.empty();
        Optional<Double> direction = Optional.empty();
        if (position == Position.LEFT) {
            turn = Optional.of(-3.0d);
            direction = Optional.of(RIGHT + 0.1);
        } else if (position == Position.RIGHT ){
            turn = Optional.of(3.0d);
            direction = Optional.of(LEFT - 0.1);
        }

        if (turn.isPresent()) {
            double firstEnd = runtime.time() + 0.75d;
            while (runtime.time() < firstEnd) {
                robot.mecanumDrive.drive(0.0d, turn.get(), 0.0d);
            }

            double secondEnd = runtime.time() + 0.5d;
            while (runtime.time() < secondEnd) {
                robot.mecanumDrive.drive(BACKWARDS, 0.0d, 3.0d);
            }
        }

        stopMecanumDrive();
    }

    private void dropPixel() {
        robot.grip.toggleLeft();

        doUpdates(2.0d);

        robot.arm.setPosition(500.0d);


        double end = runtime.time() + 1.0d;

        while (runtime.time() < end) {
            robot.mecanumDrive.drive(BACKWARDS, 0.0d, 1.0d);
        }

        stopMecanumDrive();
    }

    private void doUpdates(double time) {
        double end = runtime.time() + time;
        while (runtime.time() < end) {
            update();
        }
    }
}
