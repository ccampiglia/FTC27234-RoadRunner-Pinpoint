package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPose = new Pose2d(24, -63, Math.toRadians(90.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                //.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                //.forward(30)
                //.turn(Math.toRadians(90))
                //.forward(30)
                //.turn(Math.toRadians(90))
                //.forward(30)
                //.turn(Math.toRadians(90))
                //.forward(30)
                //.turn(Math.toRadians(90))
                //.build());

                //**** Move forward and spline behind first block
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialPose)
                        .splineToConstantHeading(new Vector2d(37.00, -40.00), Math.toRadians(90))
                        .lineTo(new Vector2d(37.00, 0.0))
                        .splineToConstantHeading(new Vector2d(48.00, 00.00), Math.toRadians(90))
                        //**** Push block back to observation zone
                        .lineTo(new Vector2d(48.00, -60))
                        //**** Move straight forward then spline behind second block
                        //.lineTo(new Vector2d(48.00, -5.0))
                        .lineToLinearHeading(new Pose2d(40,-5, Math.toRadians(270)))
                        //.splineToConstantHeading(new Vector2d(54.00, 0.00), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(54.00, 0.00), Math.toRadians(270))

                        //**** Push block back to observation zone
                        .lineTo(new Vector2d(62.00, -60))
                        //**** Move straight forward then spline behind third block
                        .lineTo(new Vector2d(62.00, 5))
                        .splineToConstantHeading(new Vector2d(72.00, 10.00), Math.toRadians(270))
                        //**** Push block back to observation zone
                        .lineTo(new Vector2d(72.00, -50))
                        .splineToConstantHeading(new Vector2d(60.00, -50.00), Math.toRadians(270))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}