package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name="blue_alliance_spin_duck_depot_park", group="Linear Opmode")
public class blue_alliance_spin_duck_depot_park extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.4, 61, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        double initialTime = 0;

        drive.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    drive.arm.setTargetPosition(400);
                    drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.arm.setPower(0.4);
                })
                .addTemporalMarker(2, () -> {
                    drive.arm.setPower(0);
                })
                .waitSeconds(2)
                .strafeRight(3)
                .back(26)
                .addTemporalMarker(3.62, () -> {
                    drive.spinner.setPower(0.4);
                })
                .waitSeconds(2)
                .addTemporalMarker(5.62, () -> {
                    drive.spinner.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(-62, 24))
                .addTemporalMarker(6.64, () -> {
                    drive.arm.setTargetPosition(3000);
                    drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.arm.setPower(0.4);
                })
                .addTemporalMarker(8.64, () -> {
                    drive.arm.setPower(0);
                })
                .addTemporalMarker(8.8, () -> {
                    drive.intake.setPower(-1);
                })
                .addTemporalMarker(9.8, () -> {
                    drive.intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(-31, 23))
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(-61, 38))
                .splineToConstantHeading(new Vector2d(46, 63), Math.toRadians(0))


                .build();


        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);

    }


}
