package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.helpers.FloodgateMotor
import org.firstinspires.ftc.teamcode.helpers.LogTelemetry
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.control.NullLocalizer
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.helpers.interp
import org.firstinspires.ftc.teamcode.helpers.registerTunable
import org.firstinspires.ftc.teamcode.rr.Localizer
import java.lang.Math.toRadians
import kotlin.math.abs
import kotlin.math.absoluteValue

class Shooter(hardwareMap: HardwareMap, val localizer: Localizer = NullLocalizer()): Mechanism {
    companion object {
        var p = 0.0
        var i = 0.0
        var d = 1.0e-3
        var kV = 2.2e-4
        var kA = 0.0
        var kStatic = 0.0
        var readyThresholdRpm = 50.0 // could be 100.0
        /*
        TABLE
        1800 at 90
        2200 at 140
         */

        // 1800 at 45 deg? at >6 ft
        var firingRpms = mapOf(Pair(90.0,1800.0),Pair(140.0,2200.0))
        var blueGoal = Vector2d(-72.0, -80.0)
        var redGoal = Vector2d(-72.0, 80.0)
        var shooterX = 0.0
        var shooterY = 0.0
        var shooterZ = 0.0
        var shooterHeading = toRadians(180.0)

        init {
            registerTunable(::p,"Shooter")
            registerTunable(::i,"Shooter")
            registerTunable(::d,"Shooter")
            registerTunable(::kV,"Shooter")
            registerTunable(::kA,"Shooter")
            registerTunable(::kStatic,"Shooter")
            registerTunable(::readyThresholdRpm,"Shooter")
            //registerTunable(::firingRpms,"Shooter")
            registerTunable(::blueGoal,"Shooter")
            registerTunable(::redGoal,"Shooter")
            registerTunable(::shooterX,"Shooter")
            registerTunable(::shooterY,"Shooter")
            registerTunable(::shooterZ,"Shooter")
            registerTunable(::shooterHeading,"Shooter")
        }
    }

    val targetGoal get() = if (PoseStorage.currentTeam == PoseStorage.Team.RED) redGoal else blueGoal

    val targetHeading get() = (targetGoal - localizer.pose.position).angleCast() + toRadians(180.0)

    val distance get() = (targetGoal - localizer.pose.position).norm()


    val pid get() = PIDFController.PIDCoefficients(p,i,d)
    var firingRpmOffset = 0.0

    // TODO interplut
    val autoFiringRpm get() = firingRpms.interp(distance) + firingRpmOffset//firingRpms.entries.sortedBy { (key, _) -> abs(key - distance) }[0].value + firingRpmOffset


    val shooter1: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "shooter1")
    val shooter2: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "shooter2")

    init {
        shooter2.direction = Direction.REVERSE
    }

    val shooter1PID =
        PIDFController(pid,kV,kA,kStatic)
    val shooter2PID =
        PIDFController(pid,kV,kA,kStatic)

    val shooter1rpm get() = shooter1.velocity * 2
    val shooter2rpm get() = shooter2.velocity * 2

    var targetRpmGen = { 0.0 }

    val targetRpm
        get() = targetRpmGen()

    val ready get() = (shooter1rpm-targetRpm).absoluteValue < readyThresholdRpm && (shooter2rpm-targetRpm).absoluteValue < readyThresholdRpm

    override fun update(telemetry: LogTelemetry) {
        shooter1PID.targetVelocity = targetRpm
        shooter2PID.targetVelocity = targetRpm

        shooter1.power = shooter1PID.update(System.nanoTime(), 0.0, shooter1rpm)
        shooter2.power = shooter2PID.update(System.nanoTime(), 0.0, shooter2rpm) * -1

        telemetry.addData("shooter/shooter1rpm", shooter1rpm)
        telemetry.addData("shooter/shooter2rpm", shooter2rpm)
        telemetry.addData("shooter/targetRpm", targetRpm)
        telemetry.addData("shooter/ready", ready)
        telemetry.addData("shooter/distance", distance)
        telemetry.addData("shooter/targetHeading", targetHeading)
        telemetry.addData("shooter/targetGoal", targetGoal)
        telemetry.addData("shooter/localizerPose", localizer.pose)
    }

    fun spinUp() = SequentialAction(
        InstantAction {
            targetRpmGen = ::autoFiringRpm // only set firing speed once
        },
        Action {
            return@Action !ready
        }
    )

    fun spinDown() = Action {
        targetRpmGen = {0.0} // set firing speed in loop for safety/paranoia
        return@Action !ready
    }
}