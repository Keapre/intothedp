package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(7, 61, Math.toRadians(270));
        Pose2d specimenDrop = new Pose2d(3, 41, Math.toRadians(270));
        Pose2d bucketFirst = new Pose2d(54,50,Math.toRadians(260));
        Pose2d bucketSecond = new Pose2d(57,45,Math.toRadians(270));
        Pose2d park = new Pose2d(-55,-55,Math.toRadians(45));
        Pose2d bucketThird = new Pose2d(57,45,Math.toRadians(295));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .strafeTo(new Vector2d(specimenDrop.position.x,specimenDrop.position.y))
                .splineToLinearHeading(bucketFirst, Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(bucketSecond.position.x,bucketSecond.position.y),bucketSecond.heading.toDouble())
                //.strafeToLinearHeading(new Vector2d(bucketThird.position.x,bucketThird.position.y),bucketThird.heading.toDouble())
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}