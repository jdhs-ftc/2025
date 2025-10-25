package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.helpers.FakeCRServo
import org.firstinspires.ftc.teamcode.helpers.FakeDcMotorEx
import org.firstinspires.ftc.teamcode.helpers.FakeServo
import org.firstinspires.ftc.teamcode.helpers.RGBLight
import org.firstinspires.ftc.teamcode.rr.Localizer

class Robot(hardwareMap: HardwareMap, localizer: Localizer) {
    val shooter = Shooter(hardwareMap, localizer)
    val transfer: Servo = hardwareMap.servo["transfer"]

    val intake: DcMotor = hardwareMap.dcMotor["intake"]

    val hw = arrayOf(shooter)

    val light = RGBLight(hardwareMap.servo["light"])

    fun update() = hw.forEach { it.update() }

    fun updateAction() =
        Action {
            update()
            return@Action true
        }

    // TODO seems wrong (but arm transfer is cooked anyway)
    val transferShoot = 0.0
    val transferStop = 0.30

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
}