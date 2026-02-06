package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.jakewharton.threetenabp.AndroidThreeTen.init
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.LogTelemetry
import org.firstinspires.ftc.teamcode.helpers.RGBLight
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.rr.MecanumDrive
import kotlin.jvm.java

class Robot(hardwareMap: HardwareMap, val drive: MecanumDrive) {
    val telemetry = LogTelemetry("Robot/")
    val shooter = Shooter(hardwareMap, drive.localizer)
    val transfer: DcMotor = hardwareMap.dcMotor["transfer"]
    val transferArm: Servo = hardwareMap.servo["transferArm"]

    val intake: DcMotor = hardwareMap.dcMotor["intake"]



    val light = RGBLight(hardwareMap.servo["light"])

    val intakeSensor: DigitalChannel = hardwareMap.digitalChannel["intakeSensor"]

    val shooterSensor: DigitalChannel = hardwareMap.digitalChannel["shooterSensor"]

    val laserCombo = LaserCombo(intakeSensor, shooterSensor)

    val hw = arrayOf(shooter,laserCombo)

    fun update() {
        hw.forEach { it.update(telemetry) }
        telemetry.addData("intakeSensor", intakeSensor.state)
        telemetry.addData("shooterSensor", shooterSensor.state)
        telemetry.addData("light",light.color)
        telemetry.addData("transferSpeed",transferSpeed)
        telemetry.addData("intakeSpeed",intakePower)
        telemetry.update()
    }

    fun updateAction() =
        Action {
            update()
            return@Action true
        }

    val transferStop = 0.0
    val transferShoot = 1.0


    var transferSpeed = 0.0
        set(value) {
            transfer.power = value
            transferArm.position = value
            field = value
        }

    fun toggleTransfer() {
        transferSpeed = if (transferSpeed == transferStop) {
            transferShoot
        } else {
            transferStop
        }
    }

    fun transferFire() = SequentialAction(
        InstantAction { transferSpeed = transferShoot }
    )

    val intakeRun = 1.0
    val intakeStop = 0.0
    val intakeReverse = -1.0

    var intakePower = 0.0
        set(value) {
            intake.power = value
            field = value
        }

    fun toggleIntake() {
        intakePower = if (intakePower == intakeStop) {
            intakeRun
        } else {
            intakeStop
        }

    }

    fun runIntake() = InstantAction { intakePower = intakeRun }
    fun stopIntake() = InstantAction { intakePower = intakeStop }

    val headingPID = PIDFController.PIDCoefficients(1.0, 0.0, 0.0)
    val headingController = PIDFController(headingPID)
    init {
        headingController.setInputBounds(-Math.PI, Math.PI)
    }

    fun autoAim() = Action {
        drive.updatePoseEstimate()
        headingController.targetPosition = shooter.targetHeading.toDouble()
        val headingInput = headingController.update(drive.localizer.pose.heading.toDouble())
        drive.setDrivePowers(
            PoseVelocity2d(
                Vector2d(0.0, 0.0),
                headingInput
            )
        )
        true
    }

    fun fireOnce() = SequentialAction(
        shooter.spinUp(),
        transferFire(),
        RaceParallelAction(SleepAction(0.25), shooter.waitTillFire()),
        InstantAction { transferSpeed = transferStop }
    )

    fun autoFire() = RaceParallelAction(
        autoAim(),
                SequentialAction(
                    shooter.spinUp(),
                    fireOnce(),
                    fireOnce(),
                    fireOnce(),
                    shooter.spinDown(),
                ))
}