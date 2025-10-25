package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.lang.Math.toRadians


fun main() {
    val meepMeep = MeepMeep(800)

    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constracints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .build()

    val shootPose = Pose2d(-12.0, 12.0, toRadians(315.0))

    myBot.runAction(
        myBot.drive.actionBuilder(Pose2d(61.0, 12.0, toRadians(180.0)))
            .setTangent(toRadians(180.0))
            .splineToSplineHeading(shootPose,toRadians(180.0))
            // fire
            .waitSeconds(0.5)
            .setTangent(toRadians(90.0))
            // start intake
            .splineToSplineHeading(Pose2d(-10.0, 45.0, toRadians(90.0)), toRadians(90.0))
            .splineToSplineHeading(shootPose, toRadians(270.0))
            // fire
            .waitSeconds(0.5)
            .setTangent(toRadians(45.0))
            // start intake
            .splineToSplineHeading(Pose2d(14.0,45.0,toRadians(90.0)),toRadians(90.0))
            .splineToSplineHeading(shootPose, toRadians(240.0))
            // fire
            .waitSeconds(0.5)
            .setTangent(toRadians(30.0))
            // start intake
            .splineToSplineHeading(Pose2d(40.0,45.0,toRadians(90.0)),toRadians(90.0))
            .splineToSplineHeading(shootPose, toRadians(210.0))
            // fire

            .build()
    )

    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_BLACK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}