package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.rr.Localizer

class Robot(hardwareMap: HardwareMap, localizer: Localizer) {
    val shooter = Shooter(hardwareMap, localizer)

    val hw = arrayOf(shooter)

    fun update() = hw.forEach { it.update() }

    fun updateAction() =
        Action {
            update()
            return@Action true
        }


    val transfer1 = hardwareMap.crservo["transfer1"]

    val transfers = arrayOf(transfer1)
}