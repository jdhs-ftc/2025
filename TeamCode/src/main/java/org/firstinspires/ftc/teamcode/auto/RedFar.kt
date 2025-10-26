package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.atag.AprilTagLocalizer
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.mechanisms.Robot
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import java.lang.Math.toRadians
@Autonomous(preselectTeleOp = "00 Teleop Field Centric")
class RedFar: LinearOpMode() {
    override fun runOpMode() {
        AprilTagLocalizer.enabled = false
        val startPose = Pose2d(61.0, 12.0, toRadians(180.0))
        val shootPose = Pose2d(-14.0, 14.0, toRadians(330.0))

        val drive = MecanumDrive(hardwareMap,startPose)
        val robot = Robot(hardwareMap,drive)

        PoseStorage.currentTeam = Team.RED

        val traj = drive.actionBuilder(startPose)
            .afterTime(0.1, robot.runIntake()) // just run the intake continuously
            .setTangent(toRadians(180.0))
            .splineToSplineHeading(shootPose, toRadians(180.0))
            .stopAndAdd (robot.autoFire())
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(-10.0, 45.0, toRadians(90.0)), toRadians(90.0))
            .splineToSplineHeading(shootPose, toRadians(270.0))
            // fire
            .stopAndAdd(robot.autoFire())
            .setTangent(toRadians(45.0))
            // start intake
            .splineToSplineHeading(Pose2d(14.0, 45.0, toRadians(90.0)), toRadians(90.0))
            .splineToSplineHeading(shootPose, toRadians(240.0))
            .stopAndAdd(robot.autoFire())
            .setTangent(toRadians(30.0))
            // start intake
            .splineToSplineHeading(Pose2d(36.0, 45.0, toRadians(90.0)), toRadians(90.0))
            .splineToSplineHeading(shootPose, toRadians(210.0))
            .stopAndAdd (robot.autoFire())

            .build()


        waitForStart()

        runBlocking(
            RaceParallelAction(
                SequentialAction(traj, robot.autoFire()),
                robot.updateAction()
            )
        )

        PoseStorage.currentPose = drive.localizer.pose

    }
}