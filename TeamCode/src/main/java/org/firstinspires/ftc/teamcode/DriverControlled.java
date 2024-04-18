/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver Controlled", group="CENTERSTAGE")

public class DriverControlled extends OpMode {
    private Robot robot;

    private final ElapsedTime runtime = new ElapsedTime();
    
    private double lastFrame;

    private boolean rightBumper;
    private boolean leftBumper;

    @Override
    public void init() {
        robot = new Robot(this);

        rightBumper = false;
        leftBumper = false;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
        lastFrame = runtime.seconds();

        robot.start();
    }

    @Override
    public void loop() {
        double currentFrame = runtime.seconds();
        double delta = currentFrame - lastFrame;
        lastFrame = currentFrame;

        driveMecanumDrive();

        driveGrip(delta);

        driveArm(delta);

        telemetry.addData("Run Time", "%.2f", currentFrame);
        telemetry.update();
    }

    private void driveArm(double delta) {
        double armDrive = gamepad2.right_trigger - gamepad2.left_trigger;
        robot.arm.drive(delta, armDrive);

        double offset = (gamepad2.dpad_up ? 1.0 : 0.0) - (gamepad2.dpad_down ? 1.0 : 0.0) * 100.0 * delta;
        robot.arm.changeOffset(offset);
    }

    private void driveGrip(double delta) {
        double pitch = gamepad2.left_stick_y * delta;
        double yaw = -gamepad2.right_stick_x  / 3;
        robot.grip.drive(pitch, yaw);

        if (gamepad2.left_bumper && !leftBumper) {
            robot.grip.toggleLeft();
        }
        leftBumper = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !rightBumper) {
            robot.grip.toggleRight();
        }
        rightBumper = gamepad2.right_bumper;
    }

    private void driveMecanumDrive() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double dir = Math.atan2(y, x);
        if (gamepad1.left_stick_button) {
            dir = Math.round(dir / (Math.PI / 2)) * (Math.PI / 2);
        }
        double mag = Math.sqrt(x * x + y * y) * 3.0d;
        double turn = gamepad1.right_stick_x * 3.0d;

        robot.mecanumDrive.drive(dir, turn, mag);
    }

}
