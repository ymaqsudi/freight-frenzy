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

@Autonomous(name="red_alliance_spin_duck_depot_park", group="Linear Opmode")
public class red_alliance_spin_duck_depot_park extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.4, -60.6, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double initialTime = 0;

        drive.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(0, () -> {
                    drive.arm.setTargetPosition(500);
                    drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.arm.setPower(0.4);
                })
                .addTemporalMarker(2, () -> {
                    drive.arm.setPower(0);
                })
                .addTemporalMarker(4.33, () -> {
                    drive.spinner.setPower(0.4);
                })
                .addTemporalMarker(7.85, () -> {
                    drive.spinner.setPower(0);
                })

                .waitSeconds(2)
                .forward(2)
                .strafeLeft(24)
                .waitSeconds(3.5)
                .lineToLinearHeading(new Pose2d(-64, -36, Math.toRadians(90)))
                .waitSeconds(3)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);

    }


}
