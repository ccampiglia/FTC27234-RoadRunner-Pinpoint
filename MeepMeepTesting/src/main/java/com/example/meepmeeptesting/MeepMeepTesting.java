package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPose = new Pose2d(24, -63, Math.toRadians(90.00));
        double velocityOverride = 100.0;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                ///**** Move forward and spline behind first block and push back to OZ
                ///**** Move forward and spline behind first block and push back to OZ
                //.splineToLinearHeading(new Pose2d(40.0, -40.0, Math.toRadians(90)), Math.toRadians(90))
                //.lineToY(15.0)
                //.strafeToLinearHeading(new Vector2d(53, 15), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride))

                ///Place First Specimen from start
                .strafeToLinearHeading(new Vector2d(0, -33.0), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))

                ///Score First Specimen
                .waitSeconds(.2)

                ///Set up for Collection
                //.stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_SPECIMEN))
                //.stopAndAdd(wrist.wristCollect())

                ///Go to Observation Zone to collect Second specimen
                .strafeToLinearHeading(new Vector2d(52, -42), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride))

                ///Grab Second Specimen on wall
                .waitSeconds(.3)

                ///Back up and Move to Submersible
                .strafeToLinearHeading(new Vector2d(-10, -33.0), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))

                ///Score Second Specimen
                .waitSeconds(.3)

                ///**** Move to spline behind first block and push back to OZ
                .strafeToLinearHeading(new Vector2d(35, -35.0), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))
                //.splineToLinearHeading(new Pose2d(40.0, -40.0, Math.toRadians(90)), Math.toRadians(90))
                //.lineToX(15.0)
                .strafeToLinearHeading(new Vector2d(53, 15), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))

                ///Push Block and Move back to OZ
                //.lineToY(-57)
                .strafeToLinearHeading(new Vector2d(53, -57), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))
                .waitSeconds(.5)
                .build()

        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}