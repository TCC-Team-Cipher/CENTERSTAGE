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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver Controlled", group="CENTERSTAGE")

public class DriverControlled extends OpMode
{
    private SoftwareServo grip;
    private SoftwareServo rotate;

    private MecanumDrive mecanumDrive;

    private ElapsedTime runtime = new ElapsedTime();
    
    private double lastFrame;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap);

        grip = new SoftwareServo(hardwareMap, "grip", 0.25, 0.5);
        rotate = new SoftwareServo(hardwareMap, "rotate", 0.65, 1);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        lastFrame = runtime.seconds();
    }

    @Override
    public void loop() {
        double currentFrame = runtime.seconds();
        double delta = currentFrame - lastFrame;
        lastFrame = currentFrame;

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double dir = Math.atan2(y, x);
        double mag = Math.sqrt(x * x + y * y);
        double turn = gamepad1.right_stick_x;

        mecanumDrive.drive(dir, mag, turn);

        grip.drive(delta, gamepad2.left_stick_x);
        rotate.drive(delta, gamepad2.left_stick_y);

        telemetry.addData("Run Time", runtime.toString());

        telemetry.addData("Grip Position", grip.position);
        telemetry.addData("Rotate Position", rotate.position);
    }

    @Override
    public void stop() {
    }

}
