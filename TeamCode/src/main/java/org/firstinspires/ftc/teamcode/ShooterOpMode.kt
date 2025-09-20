package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController

@TeleOp
class ShooterOpMode: OpMode() {
    lateinit var shooter1: DcMotorEx
    lateinit var shooter2: DcMotorEx

    override fun init() {
        shooter1 = hardwareMap.get(DcMotorEx::class.java, "shooter1")
        shooter2 = hardwareMap.get(DcMotorEx::class.java, "shooter2")
        shooter1.mode = RUN_WITHOUT_ENCODER
        shooter2.mode = RUN_WITHOUT_ENCODER
        shooter2.direction = Direction.REVERSE
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    }

    val shooter1PID = PIDFController(PIDFController.PIDCoefficients(0.0,0.0,0.00025),0.00017,0.0,0.0)
    val shooter2PID = PIDFController(PIDFController.PIDCoefficients(0.0,0.0,0.00025),0.00017,0.0,0.0)

    var targetRpm = 0.0

    override fun loop() {



        val shooter1rpm = shooter1.velocity * 2
        val shooter2rpm = shooter2.velocity * 2

        if (gamepad1.dpadUpWasPressed()) targetRpm += 500.0

        if (gamepad1.dpadDownWasPressed()) targetRpm -= 500.0

        if (gamepad1.crossWasPressed()) targetRpm = 3000.0

        if (gamepad1.circleWasPressed()) targetRpm = -3000.0

        if (gamepad1.squareWasPressed()) targetRpm = 0.0
        shooter1PID.targetVelocity = targetRpm
        shooter2PID.targetVelocity = targetRpm

        shooter1.power = shooter1PID.update(System.nanoTime(),0.0,shooter1rpm)
        shooter2.power = shooter2PID.update(System.nanoTime(),0.0,shooter2rpm)

        telemetry.addData("Target Velocity", targetRpm)
        telemetry.addData("Shooter 1 Velocity",shooter1rpm)
        telemetry.addData("Shooter 2 Velocity",shooter2rpm)
        telemetry.addData("shooter 1 power",shooter1.power)
        telemetry.addData("shooter 2 power",shooter2.power)
        telemetry.update()
    }
}