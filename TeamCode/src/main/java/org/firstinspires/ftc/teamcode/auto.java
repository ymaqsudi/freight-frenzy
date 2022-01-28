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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="auto", group="Linear Opmode")
public class auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePushbot hardware = new HardwarePushbot();

    final int ticks = 537;
    double circumference = 3.14*2.5;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap);

        hardware.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            driveForwardOrBackward(.2, 10);
            turnPID(90);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }



    public void armUp() {
        hardware.arm.setPower(.5);
        sleep(3000);
        if (hardware.arm.isBusy()) {

        }
        hardware.arm.setPower(0.1);
    }

    public void armDown() {
        hardware.arm.setPower(.01);
        sleep(500);
        hardware.arm.setPower(0.1);
    }

    public void driveForwardOrBackward(double power, double distance) {
        hardware.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotationsNeeded = distance/circumference;
        int encoderDrivingTarget = (int) (rotationsNeeded * ticks);

        hardware.left.setTargetPosition(encoderDrivingTarget);
        hardware.right.setTargetPosition(encoderDrivingTarget);

        hardware.left.setPower(power);
        hardware.right.setPower(power);

        hardware.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.left.isBusy() || hardware.right.isBusy()) {
            // do nothing while running
            telemetry.addData("Path", "Driving");
        }

        hardware.left.setPower(0);
        hardware.right.setPower(0);
    }

    public void intake(int time) {
        hardware.intake.setPower(1);
        sleep(time * 1000);
    }

    public void outtake(int time) {
        hardware.intake.setPower(-1);
        sleep(time * 1000);
    }

    public void resetAngle() {
        lastAngles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        Orientation orientation = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;


        // Normalizing angle
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;

        telemetry.addData("gyro", orientation.firstAngle);

        return currAngle;

    }

    public void turn (double degrees) {

        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3: 0.3);
            hardware.setMotorPower(motorPower, -motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        hardware.setAllPower(0);
    }

    public void turnTo(double degrees) {
        Orientation orientation = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn (error);
    }

    public double getAbsoluteAngle() {
        return hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    void turnToPID (double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, .05, 0, 0.003);
        while (opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1) {
            double motorPower = pid.update(getAbsoluteAngle());
            hardware.setMotorPower(motorPower, -motorPower);
        }
        hardware.setAllPower(0);
    }

    void turnPID (double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }


}
