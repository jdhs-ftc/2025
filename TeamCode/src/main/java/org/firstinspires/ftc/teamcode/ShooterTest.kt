package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.helpers.ActionOpMode
import org.firstinspires.ftc.teamcode.helpers.LogTelemetry
import org.firstinspires.ftc.teamcode.mechanisms.Shooter

@TeleOp
class ShooterTest: ActionOpMode() {
    override fun runOpMode() {
        val shooter = Shooter(hardwareMap)
        val telemetry = LogTelemetry("/Shooter/")
        waitForStart()
        while (opModeIsActive()) {
            shooter.update(telemetry)
            if (gamepad1.aWasPressed()) run(shooter.spinUp())
            if (gamepad1.bWasPressed()) run(shooter.spinDown())
            updateAsync()
        }
    }


}