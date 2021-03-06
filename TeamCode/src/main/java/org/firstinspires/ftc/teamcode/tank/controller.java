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

package org.firstinspires.ftc.teamcode.tank;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="controller")
@Disabled
public class controller extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePushbot hardware = new HardwarePushbot();
    auto_methods methods = new auto_methods(hardware);

    double spinSpeed = -.7;


    @Override
    public void runOpMode() {
        hardware.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hardware.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;

            // manual mode
            if (!gamepad1.x) {
                hardware.left.setPower((y + x));
                hardware.right.setPower((y - x));
            } else {
                hardware.left.setPower((y + x)/3);
                hardware.right.setPower((y - x)/3);
            }

            // arm up and down
            if (gamepad1.right_stick_y < 0 || gamepad2.right_stick_y < 0) {
                hardware.arm.setPower(.5);
            } else if (gamepad1.right_stick_y > 0 || gamepad2.right_stick_y > 0) {
                hardware.arm.setPower(.01);
            }
            else {
                hardware.arm.setPower(0.1);
            }

            // intake
            if (gamepad1.right_trigger > 0) {
                hardware.intake.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                hardware.intake.setPower(-1);
            } else {
                hardware.intake.setPower(0);
            }


            if (gamepad1.dpad_down) {
                spinSpeed -= .001;
                sleep(200);
            } else if (gamepad1.dpad_up) {
                spinSpeed += .001;
                sleep(200);

            }

            //  spinner
            if (gamepad1.right_bumper) {
                hardware.spinner.setPower(-.55);
            } else {
                hardware.spinner.setPower(0);
            }


            // information to print to the phones
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("position", hardware.arm.getCurrentPosition());
            telemetry.addData("spin_speed", spinSpeed);
            telemetry.addData("right_speed", hardware.right.getPower());
            telemetry.addData("left_speed", hardware.left.getPower());



            telemetry.update();

        }
    }
}
