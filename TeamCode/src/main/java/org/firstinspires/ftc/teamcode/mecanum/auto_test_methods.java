package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class auto_test_methods {

    hardware_mecanum hardware = new hardware_mecanum();

    public auto_test_methods(hardware_mecanum hw) {
        hardware = hw;
    }

    static final int ticks = 560;
    static double circumference = 3.14*3.54331;

    public void driveForwardOrBackward(double power, double distance) {
        hardware.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotationsNeeded = distance/circumference;
        int encoderDrivingTarget = (int) (rotationsNeeded * ticks);

        hardware.frontLeft.setTargetPosition(encoderDrivingTarget);
        hardware.frontRight.setTargetPosition(encoderDrivingTarget);
        hardware.backLeft.setTargetPosition(encoderDrivingTarget);
        hardware.backRight.setTargetPosition(encoderDrivingTarget);

        hardware.frontLeft.setPower(power);
        hardware.frontRight.setPower(power);
        hardware.backLeft.setPower(power);
        hardware.backRight.setPower(power);

        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.frontRight.isBusy() || hardware.frontLeft.isBusy()) {
            // do nothing while running

        }

        hardware.frontRight.setPower(0);
        hardware.frontLeft.setPower(0);
        hardware.backRight.setPower(0);
        hardware.backLeft.setPower(0);

        hardware.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
