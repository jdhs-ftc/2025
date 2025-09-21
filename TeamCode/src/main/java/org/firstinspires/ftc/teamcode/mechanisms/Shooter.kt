package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.roadrunner.Action
import com.jakewharton.threetenabp.AndroidThreeTen.init
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.helpers.registerTunable
import kotlin.math.absoluteValue

class Shooter(hardwareMap: HardwareMap): Mechanism {
    companion object {
        var p = 0.0
        var i = 0.0
        var d = 1.0e-3
        var kV = 2.2e-4
        var kA = 0.0
        var kStatic = 0.0
        var readyThresholdRpm = 50.0 // could be 100.0
        var firingRpm = 3000.0

        init {
            registerTunable(::p,"Shooter")
            registerTunable(::i,"Shooter")
            registerTunable(::d,"Shooter")
            registerTunable(::kV,"Shooter")
            registerTunable(::kA,"Shooter")
            registerTunable(::kStatic,"Shooter")
            registerTunable(::readyThresholdRpm,"Shooter")
        }
    }

    var pid = PIDFController.PIDCoefficients(p,i,d)

    val shooter1: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "shooter1")
    val shooter2: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "shooter2")

    init {
        shooter1.mode = RUN_WITHOUT_ENCODER
        shooter2.mode = RUN_WITHOUT_ENCODER
        shooter2.direction = Direction.REVERSE
    }

    val shooter1PID =
        PIDFController(pid,kV,kA,kStatic)
    val shooter2PID =
        PIDFController(pid,kV,kA,kStatic)

    val shooter1rpm get() = shooter1.velocity * 2
    val shooter2rpm get() = shooter2.velocity * 2

    var targetRpm = 0.0
        set(value) {
            shooter1PID.targetVelocity = value
            shooter2PID.targetVelocity = value
            field = value
        }

    val ready get() = (shooter1rpm-targetRpm).absoluteValue < readyThresholdRpm && (shooter2rpm-targetRpm).absoluteValue < readyThresholdRpm

    override fun update() {
        shooter1PID.targetVelocity = targetRpm
        shooter2PID.targetVelocity = targetRpm

        shooter1.power = shooter1PID.update(System.nanoTime(), 0.0, shooter1rpm)
        shooter2.power = shooter2PID.update(System.nanoTime(), 0.0, shooter2rpm)
        /*

        telemetry.addData("Target Velocity", targetRpm)
        telemetry.addData("Shooter 1 Velocity", shooter1rpm)
        telemetry.addData("Shooter 2 Velocity", shooter2rpm)
        telemetry.addData("shooter 1 power", shooter1.power)
        telemetry.addData("shooter 2 power", shooter2.power)
        telemetry.update()

         */
    }

    fun spinUp() = Action {
        targetRpm = firingRpm
        return@Action !ready
    }

    fun spinDown() = Action {
        targetRpm = 0.0
        return@Action !ready
    }
}