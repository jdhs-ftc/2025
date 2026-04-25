package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
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

    val startPose = Pose2d(72.0, 12.0, toRadians(180.0))
    val shootPose = Pose2d(-20.0, 14.0, toRadians(320.0))

    myBot.runAction(
        // CLOSE AUTO
        myBot.drive.actionBuilder(startPose)
            //.afterTime(0.1, robot.runIntake()) // just run the intake continuously
            .setTangent(toRadians(180.0))
            .splineToSplineHeading(shootPose, toRadians(180.0))
            //.stopAndAdd(robot.autoFire())
            .setTangent(toRadians(0.0))
            .splineToLinearHeading(Pose2d(-10.0, 14.0, toRadians(90.0)), toRadians(0.0))
            .endTrajectory()
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(-10.0, 40.0, toRadians(90.0)), toRadians(90.0))
            .endTrajectory()
            .setTangent(toRadians(270.0))
            .splineToSplineHeading(shootPose, toRadians(270.0))
            //.afterTime(0.1, InstantAction { MecanumDrive.preciseEnd = true })
            // fire
            //.stopAndAdd(robot.autoFire())
            .setTangent(toRadians(45.0))
            // start intake
            .splineToSplineHeading(Pose2d(15.0, 20.0, toRadians(90.0)), toRadians(90.0))
            .endTrajectory()
            .splineToSplineHeading(Pose2d(15.0, 40.0, toRadians(90.0)), toRadians(90.0))
            .endTrajectory()
            .setTangent(toRadians(180.0))
            .splineToSplineHeading(shootPose, toRadians(240.0))
            //.afterTime(0.1, InstantAction { MecanumDrive.preciseEnd = true })
            //.stopAndAdd(robot.autoFire())
            .setTangent(toRadians(30.0))
            // start intake
            .splineToSplineHeading(Pose2d(38.0, 20.0, toRadians(90.0)), toRadians(90.0))
            .endTrajectory()
            .splineToSplineHeading(Pose2d(38.0, 40.0, toRadians(90.0)), toRadians(90.0))
            .setTangent(toRadians(270.0))
            .splineToSplineHeading(shootPose, toRadians(210.0))
            //.afterTime(0.1, InstantAction { MecanumDrive.preciseEnd = true })
            //.stopAndAdd(robot.autoFire())
            .strafeTo(Vector2d(-35.0, 14.0))


            .build()
    )

    meepMeep.setBackground(Background.FIELD_DECODE_JUICE_BLACK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}