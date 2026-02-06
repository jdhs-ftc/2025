package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.pedropathing.ftc.drivetrains.Mecanum
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
        //AprilTagLocalizer.enabled = false
        AprilTagLocalizer.enabled = true
        val startPose = Pose2d(61.0, 12.0, toRadians(180.0))
        val shootPose = Pose2d(-14.0, 14.0, toRadians(320.0))

        val drive = MecanumDrive(hardwareMap,startPose)
        val robot = Robot(hardwareMap,drive)

        PoseStorage.currentTeam = Team.RED

        // REAL AS OF 2/4
        val traj = drive.actionBuilderPath(startPose)
            .afterTime(0.1, robot.runIntake()) // just run the intake continuously
            .setTangent(toRadians(180.0))
            .splineToSplineHeading(shootPose, toRadians(180.0))
            .stopAndAdd (robot.autoFire())
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(-8.0, 30.0, toRadians(90.0)), toRadians(90.0))
            .splineToSplineHeading(Pose2d(-8.0, 47.0, toRadians(90.0)), toRadians(90.0))
            .endTrajectory()
            .setTangent(toRadians(180.0))
            .splineToSplineHeading(shootPose, toRadians(270.0))
            .afterTime(0.1, InstantAction { MecanumDrive.preciseEnd = true })
            // fire
            .stopAndAdd(robot.autoFire())
            .setTangent(toRadians(45.0))
            // start intake
            .splineToLinearHeading(Pose2d(15.0, 30.0, toRadians(90.0)), toRadians(90.0))
            .endTrajectory()
            .splineToSplineHeading(Pose2d(15.0, 49.0, toRadians(90.0)), toRadians(90.0))
            .endTrajectory()
            .setTangent(toRadians(180.0))
            .splineToSplineHeading(shootPose, toRadians(240.0))
            .afterTime(0.1, InstantAction { MecanumDrive.preciseEnd = true })
            .stopAndAdd(robot.autoFire())
            .setTangent(toRadians(30.0))
            // start intake
            .splineToSplineHeading(Pose2d(34.0, 30.0, toRadians(90.0)), toRadians(90.0))
            .endTrajectory()
            .splineToSplineHeading(Pose2d(34.0, 50.0, toRadians(90.0)), toRadians(90.0))
            .splineToSplineHeading(shootPose, toRadians(210.0))
            .afterTime(0.1, InstantAction { MecanumDrive.preciseEnd = true })
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