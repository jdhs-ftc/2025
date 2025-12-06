package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.absoluteValue
import kotlin.math.sign

class FloodgateMotor(val motor: DcMotorEx): DcMotorEx by motor {
    var t = LogTelemetry("FloodgateMotor/${motor.deviceName}/")
    var lastPower = 0.0
    var targetPower = 0.0
    var powerSetTimeMs = System.nanoTime() / 1e6

    val slewRate = 0.2 // max power change per 10ms

    override fun setPower(power: Double) {
        targetPower = power
        powerSetTimeMs = System.nanoTime() / 1e6
        if ((power - lastPower) < slewRate) {
            motor.power = power
            lastPower = power
        }
    }

    fun update() {
        if (lastPower != targetPower) {
            t.write("lastPowerEqualsTargetPower",false)
            val timeSinceSetMs = (System.nanoTime() / 1e6) - powerSetTimeMs
            val powerChange = (targetPower - lastPower) / timeSinceSetMs * 10
            var interpPower = 0.0
            if (powerChange.absoluteValue > slewRate) {
                interpPower = lastPower + slewRate * powerChange.sign * (timeSinceSetMs / 10)
                motor.power = interpPower
            } else {
                motor.power = targetPower
                lastPower = targetPower
            }
            t.write("targetPower", targetPower)
            t.write("lastPower", lastPower)
            t.write("powerSetTimeMs", powerSetTimeMs)
            t.write("timeSinceSetMs", timeSinceSetMs)
            t.write("powerChange", powerChange)
            t.write("interpPower", interpPower)
        } else {
            t.write("lastPowerEqualsTargetPower", true)
        }
    }

}