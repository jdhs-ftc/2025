package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

const val SLEW_RATE = 0.2

class FloodgateMotor2(val motor: DcMotorEx): DcMotorEx by motor {
    override fun setPower(power: Double) {
            val currentPower = motor.power
            val desiredChange = power -currentPower;
            val limitedChange = desiredChange.coerceIn(-SLEW_RATE,SLEW_RATE)
        motor.power = currentPower + limitedChange;
        }
    }


