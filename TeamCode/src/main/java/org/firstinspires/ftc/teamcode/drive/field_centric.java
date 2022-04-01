package org.firstinspires.ftc.teamcode.drive;


import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="field_centric", group = "advanced")
public class field_centric extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        drive.setPoseEstimate(pose_storage.currentPose);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y/2,
                    -gamepad1.left_stick_x/2
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything
            drive.update();


            if (gamepad2.right_stick_x > 0) {
                drive.turret.setTargetPosition(drive.turret.getCurrentPosition() + 200);
                drive.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turret.setPower(.2);
            } else if (gamepad2.right_stick_x < 0) {
                drive.turret.setTargetPosition(drive.turret.getCurrentPosition() - 200);
                drive.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turret.setPower(-.2);
            } else {
                drive.turret.setPower(0);
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                drive.spinner.setPower(0.5);
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                drive.spinner.setPower(-0.5);
            } else {
                drive.spinner.setPower(0);
            }

            if (gamepad2.left_stick_y > 0) {
                drive.arm.setTargetPosition(drive.arm.getCurrentPosition() - 1000);
                drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.arm.setPower(-.7);

            } else if (gamepad2.left_stick_y < 0) {
                drive.arm.setTargetPosition(drive.arm.getCurrentPosition() + 1000);
                drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.arm.setPower(.7);
            } else {
                drive.arm.setPower(0);
            }

            if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
                drive.intake.setPower(1);

            } else if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
                drive.intake.setPower(-1);
            } else {
                drive.intake.setPower(0);
            }

            if (gamepad2.dpad_down) {
                drive.arm.setTargetPosition(350);
                drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.arm.setPower(.7);
                while (drive.arm.isBusy()) {

                }
                drive.arm.setPower(0);
            }

            if (gamepad2.dpad_up) {
                drive.arm.setTargetPosition(2600);
                drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.arm.setPower(.7);
                while (drive.arm.isBusy()) {

                }
                drive.arm.setPower(0);
            }

            if (gamepad2.dpad_left) {
                drive.turret.setTargetPosition(drive.turret.getCurrentPosition() - 353);
                drive.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turret.setPower(.2);
                while (drive.turret.isBusy()) {

                }
                drive.turret.setPower(0);
            }

            if (gamepad2.dpad_right) {
                drive.turret.setTargetPosition(drive.turret.getCurrentPosition() + 353);
                drive.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turret.setPower(.2);
                while (drive.turret.isBusy()) {

                }
                drive.turret.setPower(0);
            }

            if (gamepad1.b) {
                drive.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            if (gamepad1.x) {
                drive.arm.setTargetPosition(drive.arm.getCurrentPosition() + 1000);
                drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.arm.setPower(.7);
                while (drive.arm.isBusy()) {

                }
                drive.arm.setPower(0);

                drive.turret.setTargetPosition(-353);
                drive.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turret.setPower(.2);
                while (drive.turret.isBusy()) {

                }
                drive.turret.setPower(0);

            }

            if (gamepad1.y) {
                drive.intake.setPower(1);
                sleep(300);
                drive.intake.setPower(0);

                drive.turret.setTargetPosition(0);
                drive.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turret.setPower(.2);
                while (drive.turret.isBusy()) {

                }
                drive.turret.setPower(0);

                drive.arm.setTargetPosition(0);
                drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.arm.setPower(-.7);
                while (drive.arm.isBusy()) {

                }
                drive.arm.setPower(0);

                drive.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }





            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("arm position", drive.arm.getCurrentPosition());
            telemetry.addData("turret position", drive.turret.getCurrentPosition());

            telemetry.update();
        }
    }
}
