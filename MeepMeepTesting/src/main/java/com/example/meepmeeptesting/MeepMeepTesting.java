package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPose = new Pose2d(24, -63, Math.toRadians(90.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                ///**** Move forward and spline behind first block and push back to OZ
                .splineToLinearHeading(new Pose2d(37.0, -40.0, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(15.0)
                //.splineToConstantHeading(new Vector2d(48.00, 10.00), Math.toRadians(-90))
                .splineTo(new Vector2d(48.00, 15.00), Math.toRadians(-90))
                //.stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_SPECIMEN))
                //.stopAndAdd(wrist.wristCollect())
                //.stopAndAdd(claw.openWideClaw())
                .lineToY(-48)

                ///GrabSpecimen
                //.stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                //.stopAndAdd(claw.closeClaw())

                ///Back up and Move to Submersible
                //.stopAndAdd(arm.moveArmToPosition(ARM_SCORE_SPECIMEN))
                .lineToY(-40)
                //.stopAndAdd(slide.moveSlideToPosition(SLIDE_COLLECT))
                //.stopAndAdd(wrist.wristScoreSpecimen())
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(90.00))
                .setTangent(Math.toRadians(90.00))
                .lineToY(-30.00)

                ///Score Specimen
                //.stopAndAdd(arm.moveArmToPosition(ARM_ATTACH_SPECIMEN))
                //.stopAndAdd(claw.openSmallClaw())
                //.stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))

                ///Strafe Back for 2nd Specimen
                .strafeToLinearHeading(new Vector2d(52, -48), Math.toRadians(-90.00))
                //.stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_SPECIMEN))
                //.stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                //.stopAndAdd(wrist.wristCollect())
                //.stopAndAdd(claw.openWideClaw())

                ///GrabSpecimen
                //.stopAndAdd(claw.closeClaw())

                //Move to Submersible
                //.stopAndAdd(arm.moveArmToPosition(ARM_SCORE_SPECIMEN))
                .lineToY(-40)
                //.stopAndAdd(slide.moveSlideToPosition(SLIDE_COLLECT))
                //.stopAndAdd(wrist.wristScoreSpecimen())
                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(90.00))
                .setTangent(Math.toRadians(90.00))
                //.lineToY(-30.00)

                ///Move back to OZ
                .strafeToLinearHeading(new Vector2d(52, -50), Math.toRadians(-90.00))
                //.stopAndAdd(wrist.wristFoldedIn())
                //.stopAndAdd(claw.closeClaw())
                //.stopAndAdd(arm.moveArmToPosition(ARM_COLLAPSED_INTO_ROBOT))
                .build()

        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}