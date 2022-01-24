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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="IMU_Test", group="Linear Opmode")
public class IMU_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePushbot hardware = new HardwarePushbot();


    BNO055IMU imu;
    Orientation angles;



    @Override
    public void runOpMode() {


        hardware.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0) {
                hardware.arm.setPower(.5);
            } else if (gamepad1.b) {
                hardware.arm.setPower(.01);

            }
            else {
                hardware.arm.setPower(0.1);
            }

            if (gamepad1.a) {
                turnLeft(90, 1);
            }


        }
    }

    void turnLeft (double turnAngle, double timeoutS) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        sleep (250);
        double speed = .5;
        double oldDegreesLeft = turnAngle;
        double scaledSpeed = speed;
        double targetHeading = angles.firstAngle + turnAngle;
        double oldAngle = angles.firstAngle;

        if (targetHeading < -180) {
            targetHeading += 360;
        }

        if (targetHeading > 180) {
            targetHeading -= 360;
        }

        double degreesLeft = ((Math.signum(angles.firstAngle-targetHeading) + 1)/2) * (360-Math.abs(angles.firstAngle-targetHeading)) + (Math.signum(targetHeading-angles.firstAngle) + 1)/2*Math.abs(angles.firstAngle-targetHeading);
        resetStartTime();

        while(opModeIsActive() && time <timeoutS && degreesLeft > 1 && oldDegreesLeft - degreesLeft>=0) {
            scaledSpeed = degreesLeft/(100+degreesLeft) * speed;
            if (scaledSpeed >1) {
                scaledSpeed = 0.1;
            }
            hardware.left.setPower(scaledSpeed);
            hardware.right.setPower(-scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft=degreesLeft;
            degreesLeft = ((Math.signum(angles.firstAngle-targetHeading) + 1)/2) * (360-Math.abs(angles.firstAngle-targetHeading)) + (Math.signum(targetHeading-angles.firstAngle) + 1)/2*Math.abs(angles.firstAngle-targetHeading);
            if (Math.abs(angles.firstAngle-oldAngle) < 1) {
                speed*=1.1;
                oldAngle = angles.firstAngle;
            }

            hardware.left.setPower(0);
            hardware.right.setPower(0);

        }
    }
}