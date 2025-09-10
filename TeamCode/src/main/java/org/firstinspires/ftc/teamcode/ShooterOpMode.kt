package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

@TeleOp
class ShooterOpMode: OpMode() {
    lateinit var shooter1: DcMotorEx
    lateinit var shooter2: DcMotorEx

    override fun init() {
        shooter1 = hardwareMap.get(DcMotorEx::class.java, "shooter1")
        shooter2 = hardwareMap.get(DcMotorEx::class.java, "shooter2")
        shooter1.mode = RUN_USING_ENCODER
        shooter2.mode = RUN_USING_ENCODER
        shooter2.direction = Direction.REVERSE

    }

    var velocity = 0.0

    override fun loop() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        shooter1.setVelocity(velocity,AngleUnit.DEGREES)
        shooter2.setVelocity(velocity,AngleUnit.DEGREES)

        if (gamepad1.dpadUpWasPressed()) {
            velocity += 10.0
        }
        if (gamepad1.dpadDownWasPressed()) {
            velocity -= 10.0
        }

        telemetry.addData("Target Velocity", velocity)
        telemetry.addData("Shooter 1 Velocity",shooter1.getVelocity(AngleUnit.DEGREES))
        telemetry.addData("Shooter 2 Velocity",shooter2.getVelocity(AngleUnit.DEGREES))
        telemetry.update()
    }
}