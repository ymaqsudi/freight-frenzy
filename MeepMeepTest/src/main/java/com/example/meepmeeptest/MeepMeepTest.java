package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 60, Math.toRadians(180), Math.toRadians(180), 12.17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.4, 61, Math.toRadians(0)))
                                .waitSeconds(2)
                                .strafeRight(3)
                                .back(26)
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-62, 24))
                                .lineToConstantHeading(new Vector2d(-31, 23))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-61, 38))
                                .splineToConstantHeading(new Vector2d(46, 63), Math.toRadians(0))


                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}