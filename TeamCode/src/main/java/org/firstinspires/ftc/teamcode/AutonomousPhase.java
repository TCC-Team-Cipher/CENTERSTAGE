package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ArmConfig;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Autonomous Phase", group = "CENTERSTAGE")
public class AutonomousPhase extends LinearOpMode {
    static class Arm {
        private final Motor motor;
        private final PIDController controller;

        public Arm(HardwareMap hardwareMap) {
            this.motor = new Motor(hardwareMap, "armMotor");
            this.motor.setInverted(true);
            this.motor.setRunMode(Motor.RunMode.RawPower);
            this.motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            this.motor.resetEncoder();

            this.controller = new PIDController(ArmConfig.P, ArmConfig.I, ArmConfig.D);
            this.controller.setTolerance(ArmConfig.TOLERANCE);
        }
    }

    private Arm arm;

    private Claw left;
    private Claw right;

    public class MoveArm implements Action {
        private final double position;

        public MoveArm(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.controller.setSetPoint(this.position);
            double output = arm.controller.calculate(arm.motor.getCurrentPosition());
            if (!arm.controller.atSetPoint()) {
                arm.motor.set(output);
                return true;
            } else {
                arm.motor.set(0);
                return false;
            }
        }
    }

    public class SetClaw implements Action {
        private final Claw claw;
        private final boolean open;

        public SetClaw(Claw claw, boolean open) {
            this.claw = claw;
            this.open = open;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            this.claw.set(this.open);
            return false;
        }
    }

    @Override
    public void runOpMode() {
        DistanceSensor leftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor rightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        arm = new Arm(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-30d, 30d, Math.toRadians(90)));

        left = new Claw(hardwareMap, "left");
        right = new Claw(hardwareMap, "right");

        Action liftArm = new MoveArm(200d);

        Action initialMove = drive.actionBuilder(drive.pose)
                .lineToY(18d)
                .build();

        Action position1 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(-30, 18d), Math.toRadians(0d))
                .build();

        Action position2 = drive.actionBuilder(drive.pose)
                .build();

        Action position3 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(-30, 18d), Math.toRadians(180d))
                .build();

        Action openLeftClaw = new SetClaw(left, true);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        Actions.runBlocking(new SequentialAction(
                liftArm,
                initialMove
        ));

        Action nextAction;
        if (leftSensor.getDistance(DistanceUnit.CM) < 30d) {
            nextAction = position1;
        } else if (rightSensor.getDistance(DistanceUnit.CM) < 30d) {
            nextAction = position3;
        } else {
            nextAction = position2;
        }

        Actions.runBlocking(new SequentialAction(
                nextAction,
                openLeftClaw
        ));
    }
}
