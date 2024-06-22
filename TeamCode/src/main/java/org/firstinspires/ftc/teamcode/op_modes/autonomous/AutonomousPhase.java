package org.firstinspires.ftc.teamcode.op_modes.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ArmConfig;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.GripPitch;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

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

    public enum MarkerPosition {
        LEFT,
        RIGHT,
        CENTER
    }

    private final boolean reflect;
    private final boolean backboard;

    private Arm arm;

    private DistanceSensor leftSensor;
    private DistanceSensor rightSensor;

    private MarkerPosition markerPosition;

    private GripPitch pitch;

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

    public class CheckForMarkers implements Action {
        private static final double MAX_TIME = 5d;
        private static final double DISTANCE = 30d;

        private final double startTime;

        public CheckForMarkers() {
            this.startTime = getRuntime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (getRuntime() - startTime > MAX_TIME || markerPosition != MarkerPosition.CENTER) {
                return false;
            }

            double leftDistance = leftSensor.getDistance(DistanceUnit.CM);
            double rightDistance = rightSensor.getDistance(DistanceUnit.CM);

            telemetryPacket.put("timeRemaining", MAX_TIME - getRuntime() + startTime);
            telemetryPacket.put("leftDistance", leftDistance);
            telemetryPacket.put("rightDistance", rightDistance);

            if (leftDistance < DISTANCE) {
                markerPosition = MarkerPosition.LEFT;
            } else if (rightDistance < DISTANCE) {
                markerPosition = MarkerPosition.RIGHT;
            }

            telemetryPacket.put("markerPosition", markerPosition);

            return true;
        }
    }

    public AutonomousPhase(boolean reflect, boolean backboard) {
        this.reflect = reflect;
        this.backboard = backboard;
    }

    @Override
    public void runOpMode() {
        markerPosition = MarkerPosition.CENTER;

        pitch = new GripPitch(hardwareMap);

        leftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        PoseMap poseMap = pose -> new Pose2dDual<>(
                pose.position.x,
                this.reflect ? pose.position.y.unaryMinus() : pose.position.y,
                this.reflect ? pose.heading.inverse() : pose.heading
        );

        Pose2d initialPose = new Pose2d(backboard ? 12d : -36d, 66d, Math.toRadians(-90d));

        arm = new Arm(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw left = Claw.left(hardwareMap);
        Claw right = Claw.right(hardwareMap);

        Action initialMove = drive.actionBuilder(drive.pose, poseMap)
                .lineToY(30d)
                .build();

        Action searchingMove = drive.actionBuilder(drive.pose, poseMap)
                .lineToY(30d)
                .build();

        Action leftPosition = drive.actionBuilder(drive.pose, poseMap)
                .strafeToLinearHeading(new Vector2d(initialPose.position.x - 2d, 30d), Math.toRadians(0d))
                .build();

        Action centrePosition = drive.actionBuilder(drive.pose, poseMap)
                .lineToY(32d)
                .build();

        Action rightPosition = drive.actionBuilder(drive.pose, poseMap)
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 2d, 30d), Math.toRadians(180d))
                .build();

        Action moveToBackboard = drive.actionBuilder(drive.pose, poseMap)
                .strafeToLinearHeading(new Vector2d(30d, 30d), Math.toRadians(0d))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        Actions.runBlocking(new SequentialAction(
            new MoveArm(200d),
            initialMove,
            new CheckForMarkers()
        ));

        Action nextAction;
        switch (markerPosition) {
            case LEFT:
                nextAction = leftPosition;
                break;
            case RIGHT:
                nextAction = rightPosition;
                break;
            default:
                nextAction = centrePosition;
                break;
        }

        Actions.runBlocking(new SequentialAction(
                nextAction,
                new MoveArm(0d),
                new InstantAction(() -> left.set(true)),
                new SleepAction(1d),
                new MoveArm(200d),
                new InstantAction(() -> left.set(false))
        ));
    }
}
