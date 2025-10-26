package org.firstinspires.ftc.teamcode.mechanisms

import android.R.attr.value
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.FakeCRServo
import org.firstinspires.ftc.teamcode.helpers.FakeDcMotorEx
import org.firstinspires.ftc.teamcode.helpers.FakeServo
import org.firstinspires.ftc.teamcode.helpers.RGBLight
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.rr.Localizer
import org.firstinspires.ftc.teamcode.rr.MecanumDrive

class Robot(hardwareMap: HardwareMap, val drive: MecanumDrive) {
    val shooter = Shooter(hardwareMap, drive.localizer)
    val transfer: Servo = hardwareMap.servo["transfer"]

    val intake: DcMotor = hardwareMap.dcMotor["intake"]

    val hw = arrayOf(shooter)

    val light = RGBLight(hardwareMap.servo["light"])

    fun update() {
        hw.forEach { it.update() }
        when (intakePower) {
            intakeRun -> {
                light.color = Color.GREEN
            }
            intakeReverse -> {
                light.color = Color.RED
            }
            else -> {
                light.color = Color.BLUE
            }
        }
    }

    fun updateAction() =
        Action {
            update()
            return@Action true
        }

    // TODO seems wrong (but arm transfer is cooked anyway)
    val transferStop = 0.0
    val transferMid = 0.10
    val transferShoot = 0.30

    val transferPosList = listOf(0.0,0.10,0.30)
    val transferIndex = 0


    var transferPos = 0.0
        set(value) {
            transfer.position = value
            field = value
        }

    fun toggleTransfer() {
        transferPos = if (transferPos == transferStop) {
            transferShoot
        } else {
            transferStop
        }
    }

    fun transferFire() = SequentialAction(
        InstantAction {
            transferPos = transferMid
        },
        SleepAction(0.50),
        InstantAction {
            if (transferPos == transferMid) {
                transferPos = transferShoot
            }
        }
    )

    val intakeRun = -1.0
    val intakeStop = 0.0
    val intakeReverse = 1.0

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

    val headingPID = PIDFController.PIDCoefficients(0.5, 0.0, 0.0)
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

    fun autoFire() = RaceParallelAction(
        autoAim(),
                SequentialAction(
                    shooter.spinUp(),
                    transferFire(),
                    SleepAction(0.5),
                    InstantAction { transferPos = transferStop },
                    SleepAction(0.25),
                    transferFire(),
                    SleepAction(0.5),
                    InstantAction { transferPos = transferStop },
                    shooter.spinDown(),
                    InstantAction { transferPos = transferStop}
                ))
}