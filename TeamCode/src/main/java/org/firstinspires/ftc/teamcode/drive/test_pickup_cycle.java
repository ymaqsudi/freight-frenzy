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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="test_pickup_cycle", group="Linear Opmode")
public class test_pickup_cycle extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.4, -56.6, Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .forward(100)

                .build();


        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(1, () -> {
                    drive.intake.setPower(1);
                })

                .forward(20)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(10)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (drive.sensorRange.getDistance(DistanceUnit.MM) > 50) {
            drive.followTrajectory(traj1);
        }

        drive.followTrajectory(traj2);

    }


}
