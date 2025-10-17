package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.hardware.configuration.LynxConstants.EXPANSION_HUB_PRODUCT_NUMBER
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.helpers.ASExtraLogging
import org.firstinspires.ftc.teamcode.helpers.ActionOpMode
import org.firstinspires.ftc.teamcode.helpers.LogTelemetry
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team.BLUE
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team.RED
import org.firstinspires.ftc.teamcode.helpers.UniqueActionQueue
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.mechanisms.Robot
import org.firstinspires.ftc.teamcode.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.rr.Drawing
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import org.firstinspires.ftc.teamcode.vision.ArtifactLocator
import java.lang.Math.toRadians
import java.util.LinkedList


@TeleOp(name = "00 Teleop Field Centric")
@Config
class TeleopActions : ActionOpMode() {

    // Declare a PIDF Controller to regulate heading
    private val headingPIDJoystick = PIDFController.PIDCoefficients(0.01, 0.0, 0.0)
    private val joystickHeadingController = PIDFController(headingPIDJoystick)

    val allHubs: List<LynxModule> by lazy { hardwareMap.getAll<LynxModule>(LynxModule::class.java) }
    val controlHub by lazy {
        allHubs.find { // search through all LynxModules (chub, ex hub, shub)
            it.revProductNumber == EXPANSION_HUB_PRODUCT_NUMBER // check that it's an expansion hub not a servo hub
                    && it.isParent // check that it's directly connected over USB
                    && LynxConstants.isEmbeddedSerialNumber(it.serialNumber) // check that it's a control hub's integrated expansion hub
        } as LynxModule // ensure it's non-null
    }
    val expansionHub by lazy {
        allHubs.find { // search through all LynxModules (chub, ex hub, shub)
            it.revProductNumber == EXPANSION_HUB_PRODUCT_NUMBER // check that it's an expansion hub not a servo hub
                    && (!it.isParent // check that it's NOT connected over usb
                    || (it.isParent && !LynxConstants.isEmbeddedSerialNumber(it.serialNumber)))
            // or that it is connected over USB but isn't an integrated hub
        } as LynxModule // ensure it's non-null
    }

    val startPose: Pose2d = PoseStorage.currentPose

    lateinit var drive: MecanumDrive


    val loopTime = ElapsedTime()
    val currentGamepad1 = Gamepad()
    val currentGamepad2 = Gamepad()
    val previousGamepad1 = Gamepad()
    val previousGamepad2 = Gamepad()

    var speed = 1.0
    var targetHeading = startPose.heading
    var fieldCentric = true
    var drivingEnabled = true

    var showMotorTelemetry = true
    var showStateTelemetry = true
    var showLoopTimes = true
    var showPoseTelemetry = true
    var showCameraTelemetry = false

    val loopTimeAvg = LinkedList<Double>()
    val timeSinceDriverTurned = ElapsedTime()


    lateinit var artifactLocator: ArtifactLocator

    enum class AimMode {
        NONE,
        GOAL,
        ARTIFACT;
        fun next() = entries[(this.ordinal + 1) % entries.size]
    }
    var aimMode = AimMode.GOAL

    lateinit var robot: Robot


    override fun runOpMode() {
        ASExtraLogging.start(this)
        //  Initialization Period

        //EXPANSION_HUB = allHubs.get(1);


        // Enable Bulk Caching
        allHubs.forEach { it.bulkCachingMode == BulkCachingMode.MANUAL }

        joystickHeadingController.setInputBounds(-Math.PI, Math.PI)

        // Telemetry Init
        telemetry.msTransmissionInterval = 50
        telemetry =
            MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry, LogTelemetry())


        drive = MecanumDrive(hardwareMap, startPose)

        artifactLocator = ArtifactLocator(hardwareMap)

        robot = Robot(hardwareMap,drive.localizer)


        waitForStart()

        if (isStopRequested) return


        // Run Period
        while (opModeIsActive() && !isStopRequested) {
            ASExtraLogging.loop(this)
            // Reset measured loop time
            loopTime.reset()
            val packet = TelemetryPacket()
            // Reset bulk cache
            allHubs.forEach { it.clearBulkCache() }

            // This lets us do reliable rising-edge detection, even if it changes mid-loop
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)

            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)


            // CONTROLS

            // Gamepad 1
            // Driving Modifiers
            val padSlowMode = false // NOT MAPPED //gamepad1.right_bumper
            val padFastMode = false // NOT MAPPED
            val padResetPose = false//gamepad1.dpad_left && !previousGamepad1.dpad_left

            val padFaceDown = gamepad1.a
            val padFaceRight = gamepad1.b
            val padFaceLeft = gamepad1.x
            val padFaceUp = gamepad1.y


            // Misc/Obscure
            if (gamepad1.right_stick_button && !previousGamepad1.right_stick_button) aimMode = aimMode.next()

            // Extra Settings
            val pad1ExtraSettings = gamepad1.share
            val pad1ExTeamSwitch =
                gamepad1.dpad_left && !previousGamepad1.dpad_left // 1 rumble blue, 2 rumble red
            val pad1ExToggleFieldCentric = gamepad1.dpad_up && !previousGamepad1.dpad_up


            // Update the speed
            speed = if (padSlowMode) {
                .35
            } else if (padFastMode) {
                1.5
            } else {
                1.0 //.8; // prev 0.8
            }
            // especially in driver practice, imu drifts eventually
            // this lets them reset just in case
            if (padResetPose) {
                if (PoseStorage.currentTeam != BLUE) { // Team is declared and saved there for auto
                    drive.localizer.pose = Pose2d(
                        drive.localizer.pose.position.x,
                        drive.localizer.pose.position.y,
                        Math.toRadians(90.0)
                    )
                } else {
                    drive.localizer.pose = Pose2d(
                        drive.localizer.pose.position.x,
                        drive.localizer.pose.position.y,
                        Math.toRadians(-90.0)
                    )
                }


                targetHeading = drive.localizer.pose.heading
                gamepad1.rumbleBlips(1) // tell the driver it succeeded
            }
            // Second layer
            if (pad1ExtraSettings) {
                if (pad1ExToggleFieldCentric) {
                    fieldCentric = !fieldCentric
                    if (fieldCentric) {
                        gamepad1.rumbleBlips(2)
                    } else {
                        gamepad1.rumbleBlips(1)
                    }
                }

                if (pad1ExTeamSwitch) {
                    if (PoseStorage.currentTeam == RED) {
                        gamepad1.rumbleBlips(1)
                        PoseStorage.currentTeam = BLUE
                    } else {
                        gamepad1.rumbleBlips(2)
                        PoseStorage.currentTeam = RED
                    }
                }
            }


            // Field Centric

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            var input = Vector2d(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed
            )

            var rotationAmount = drive.localizer.pose.heading.inverse() // inverse it
            if (fieldCentric) {
                rotationAmount =
                    if (PoseStorage.currentTeam == BLUE) { // Depending on which side we're on, the forward angle from driver's perspective changes
                        rotationAmount + Math.toRadians(-90.0)
                    } else {
                        rotationAmount + Math.toRadians(90.0)
                    }
                input = rotationAmount * input // rotate the input by the rotationamount
                // (rotationAmount MUST go first here)
            }
            val controllerHeading =
                Vector2d(-gamepad1.right_stick_y.toDouble(), -gamepad1.right_stick_x.toDouble())

            var headingInput = 0.0
            if (drivingEnabled) {
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    headingInput = (gamepad1.left_trigger - gamepad1.right_trigger) * speed * 0.50
                    targetHeading = drive.localizer.pose.heading
                    timeSinceDriverTurned.reset()
                } else {
                    val baseHeading = if (PoseStorage.currentTeam == RED) {
                        Math.toRadians(90.0)
                    } else {
                        Math.toRadians(-90.0)
                    }
                    // Set the target heading for the heading controller to our desired angle
                    if (controllerHeading.norm() > 0.4) { // if the joystick is tilted more than 0.4 from the center,
                        // Cast the angle based on the angleCast of the joystick as a heading
                        targetHeading = controllerHeading.angleCast() + baseHeading
                    } else if (aimMode == AimMode.GOAL) {
                        targetHeading = robot.shooter.targetHeading
                    }


                    if (padFaceDown) {
                        targetHeading = Rotation2d.fromDouble(Math.toRadians(180.0)) + baseHeading
                    } else if (padFaceRight) {
                        targetHeading = Rotation2d.fromDouble(Math.toRadians(-90.0)) + baseHeading
                    } else if (padFaceLeft) {
                        targetHeading = Rotation2d.fromDouble(Math.toRadians(90.0)) + baseHeading
                    } else if (padFaceUp) {
                        targetHeading = Rotation2d.fromDouble(Math.toRadians(0.0)) + baseHeading
                    }

                    joystickHeadingController.targetPosition = targetHeading.toDouble()
                    // Set the desired angular velocity to the heading controller output plus angular
                    // velocity feedforward
                    if (timeSinceDriverTurned.milliseconds() > 250) {
                        if (aimMode == AimMode.ARTIFACT) {
                            headingInput += artifactLocator.correct()
                            targetHeading = drive.localizer.pose.heading
                        } else {
                            headingInput =
                                ((joystickHeadingController.update(drive.localizer.pose.heading.log())
                                        * MecanumDrive.PARAMS.kV
                                        * MecanumDrive.PARAMS.trackWidthTicks))
                        }


                    } else {
                        headingInput = 0.0
                        targetHeading = drive.localizer.pose.heading
                    }
                }
                drive.setDrivePowers(
                    PoseVelocity2d(
                        Vector2d(
                            input.x,
                            input.y
                        ),
                        headingInput
                    )
                )



                // update RR, update motor controllers

                val padSpinUp = gamepad1.right_bumper && !previousGamepad1.right_bumper
                val padSpinDown = gamepad1.left_bumper && !previousGamepad1.left_bumper
                val padLowerRpm = false //gamepad1.dpad_left && !previousGamepad1.dpad_left
                val padHigherRpm = false //gamepad1.dpad_right && !previousGamepad1.dpad_right
                val padMuchLowerRpm = gamepad1.dpad_down && !previousGamepad1.dpad_down
                val padMuchHigherRpm = gamepad1.dpad_up && !previousGamepad1.dpad_up


                val padTransferStop = gamepad1.dpad_left && !previousGamepad1.dpad_left
                val padTransferStart = gamepad1.dpad_right && !previousGamepad1.dpad_right


                if (padSpinUp) run(robot.shooter.spinUp())
                if (padSpinDown) run(robot.shooter.spinDown())

                if (padLowerRpm) robot.shooter.firingRpmOffset -= 100.0
                if (padHigherRpm) robot.shooter.firingRpmOffset += 100.0
                if (padMuchLowerRpm) robot.shooter.firingRpmOffset -= 500.0
                if (padMuchHigherRpm) robot.shooter.firingRpmOffset += 500.0

                if (padTransferStop) robot.transfers.forEach { it.power = 0.0 }
                if (padTransferStart) robot.transfers.forEach { it.power = 1.0 }



                robot.update()

                // TELEMETRY
                Drawing.drawRobot(
                    packet.fieldOverlay(),
                    drive.localizer.pose
                )

                updateAsync(packet)
                if (drivingEnabled) { // if driving's disabled, the trajectory is already updating it
                    drive.updatePoseEstimate()
                }
                //motorControl.update()
                FtcDashboard.getInstance().sendTelemetryPacket(packet)

                val loopTimeMs = loopTime.milliseconds()
                loopTimeAvg.add(loopTimeMs)
                while (loopTimeAvg.size > 1000) {
                    loopTimeAvg.removeFirst()
                }

                if (showPoseTelemetry) {
                    telemetry.addLine("--- Pose ---")
                    telemetry.addData("x", drive.localizer.pose.position.x)
                    telemetry.addData("y", drive.localizer.pose.position.y)
                    telemetry.addData("heading", drive.localizer.pose.heading.log())
                    telemetry.addData("targetHeading", targetHeading.toDouble())
                    telemetry.addData(
                        "headingDeg",
                        Math.toDegrees(drive.localizer.pose.heading.log())
                    )
                    telemetry.addData("targetHeading", Math.toDegrees(targetHeading.toDouble()))
                    telemetry.addData(
                        "poseStorageHeading",
                        Math.toDegrees(PoseStorage.currentPose.heading.toDouble())
                    )
                    telemetry.addData("headingInput", headingInput)
                    telemetry.addData("timeSinceDriverTurned", timeSinceDriverTurned.milliseconds())
                }
                if (showLoopTimes) {
                    telemetry.addLine("--- Loop Times ---")
                    telemetry.addData("loopTimeMs", loopTimeMs)
                    telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs)
                    telemetry.addData(
                        "LoopAverage ",
                        loopTimeAvg.sum() / loopTimeAvg.size
                    )
                }
                if (showMotorTelemetry) {
                    telemetry.addLine("--- Motors ---")
                }


                if (showStateTelemetry) {
                    telemetry.addLine("--- State Machine ---")
                    telemetry.addData("actions", UniqueActionQueue.runningUniqueActions)
                }
                telemetry.addData("aimMode", aimMode)
                telemetry.addData("blobPos",artifactLocator.currentPos)
                telemetry.addData("team",PoseStorage.currentTeam)
                telemetry.addData("distance",robot.shooter.distance)
                telemetry.addData("targetRpm", robot.shooter.targetRpm)
                telemetry.addData("firingRpm",robot.shooter.autoFiringRpm)
                telemetry.addData("currentRpm1",robot.shooter.shooter1rpm)
                telemetry.addData("currentRpm2",robot.shooter.shooter2rpm)
                telemetry.update()
            }
        }
    }
}

