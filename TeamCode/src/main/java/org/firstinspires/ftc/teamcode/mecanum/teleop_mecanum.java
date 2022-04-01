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

package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Controller", group="Linear Opmode")
public class teleop_mecanum extends LinearOpMode {

    hardware_mecanum hardware = new hardware_mecanum();


    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);


        hardware.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            // drivetrain code
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            hardware.frontLeft.setPower(frontLeftPower * .75);
            hardware.backLeft.setPower(backLeftPower * .75);
            hardware.frontRight.setPower(frontRightPower * .75);
            hardware.backRight.setPower(backRightPower * .75);


            if (gamepad1.left_trigger > 0) {
                hardware.intake.setPower(1);
            } else if (gamepad1.right_trigger > 0) {
                hardware.intake.setPower(-1);
            } else {
                hardware.intake.setPower(0);
            }


            // pressing x picks arm up, turns turret 90 degrees, outtakes, brings turret back to 0, brings arm down

            if (gamepad1.x) {
                hardware.arm.setTargetPosition(1000);
                hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.arm.setPower(.7);
                while (hardware.arm.isBusy()) {

                }
                hardware.arm.setPower(0);

                hardware.turret.setTargetPosition(hardware.turret.getCurrentPosition() - 353);
                hardware.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.turret.setPower(.2);
                while (hardware.turret.isBusy()) {

                }
                hardware.turret.setPower(0);

                hardware.intake.setPower(1);
                sleep(300);
                hardware.intake.setPower(0);

                hardware.turret.setTargetPosition(hardware.turret.getCurrentPosition() + 353);
                hardware.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.turret.setPower(.2);
                while (hardware.turret.isBusy()) {

                }
                hardware.turret.setPower(0);

                hardware.arm.setTargetPosition(hardware.arm.getCurrentPosition() - 1000);
                hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.arm.setPower(.7);
                while (hardware.arm.isBusy()) {

                }
                hardware.arm.setPower(0);
            }
        }





            telemetry.addData("arm position", hardware.arm.getCurrentPosition());
            telemetry.addData("turret position", hardware.turret.getCurrentPosition());
            telemetry.addData("deviceName",hardware.sensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", hardware.sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", hardware.sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", hardware.sensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", hardware.sensorRange.getDistance(DistanceUnit.INCH)));


            telemetry.update();
        }
    }



