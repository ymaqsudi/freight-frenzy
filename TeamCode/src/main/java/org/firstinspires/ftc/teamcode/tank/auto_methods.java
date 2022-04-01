package org.firstinspires.ftc.teamcode.tank;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class auto_methods {

    public auto_methods(HardwarePushbot hw) {
        hardware = hw;
    }

    static final int ticks = 560;
    static double circumference = 3.14*3.54331;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    HardwarePushbot hardware = new HardwarePushbot();







    public void intake() {
        hardware.intake.setPower(1);
    }

    public void outtake() {
        hardware.intake.setPower(-1);
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

        }

        hardware.left.setPower(0);
        hardware.right.setPower(0);

        hardware.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


        return currAngle;

    }

    public void turn (double degrees) {

        resetAngle();

        double error = degrees;

        while (Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3: 0.3);
            hardware.setMotorPower(motorPower, -motorPower);
            error = degrees - getAngle();

        }

        hardware.setAllPower(0);
    }

    public double getAbsoluteAngle() {
        return hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    void turnToPID (double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, .05, 0, 0.003);
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 1) {
            double motorPower = pid.update(getAbsoluteAngle());
            hardware.setMotorPower(motorPower, -motorPower);
        }
        hardware.setAllPower(0);
    }

    void turnPID (double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    public void barUp() {

        hardware.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.arm.setTargetPosition(100);


        hardware.arm.setPower(.5);


        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (hardware.left.isBusy() || hardware.right.isBusy()) {
           // do nothing

        }

        hardware.arm.setPower(0);


        hardware.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turncw(double power, double distance) {
        hardware.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotationsNeeded = distance / circumference;
        int encoderDrivingTarget = (int) (rotationsNeeded * ticks);

        hardware.left.setTargetPosition(-encoderDrivingTarget);
        hardware.right.setTargetPosition(encoderDrivingTarget);

        hardware.left.setPower(power);
        hardware.right.setPower(power);

        hardware.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.left.isBusy() || hardware.right.isBusy()) {
            // do nothing while running

        }

        hardware.left.setPower(0);
        hardware.right.setPower(0);

        hardware.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armUpLevel1(double power) {
        hardware.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.arm.setTargetPosition(300);

        hardware.arm.setPower(power);


        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.arm.isBusy()) {

        }

        hardware.arm.setPower(0.01);

        hardware.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armUpLevel3(double power) {
        hardware.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.arm.setTargetPosition(650);

        hardware.arm.setPower(power);


        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.arm.isBusy()) {

        }

        hardware.arm.setPower(0.01);


        hardware.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
