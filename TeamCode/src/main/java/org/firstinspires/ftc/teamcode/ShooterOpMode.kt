package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.mechanisms.Shooter

@TeleOp
class ShooterOpMode: OpMode() {
    lateinit var shooter: Shooter

    override fun init() {
        shooter = Shooter(hardwareMap)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        /*
        if (gamepad1.dpadUpWasPressed()) targetRpm += 500.0

        if (gamepad1.dpadDownWasPressed()) targetRpm -= 500.0

         */

        if (gamepad1.crossWasPressed()) shooter.spinUp().run(TelemetryPacket())

        //if (gamepad1.circleWasPressed()) targetRpm = -3000.0

        if (gamepad1.squareWasPressed()) shooter.spinDown().run(TelemetryPacket())



        shooter.update()

        telemetry.addData("Target Velocity", shooter.targetRpm)
        telemetry.addData("Shooter 1 Velocity",shooter.shooter1rpm)
        telemetry.addData("Shooter 2 Velocity",shooter.shooter2rpm)
        telemetry.addData("shooter 1 power",shooter.shooter1.power)
        telemetry.addData("shooter 2 power",shooter.shooter2.power)
        telemetry.update()
    }
}