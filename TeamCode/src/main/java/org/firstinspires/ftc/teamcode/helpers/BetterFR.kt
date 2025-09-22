package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.roadrunner.ftc.FlightRecorder

object BetterFR {
    var cachedValues = mutableMapOf<String, Any>()

    fun write(ch: String, value: Any) {
        if (!cachedValues.containsKey(ch)) {
            cachedValues[ch] = value
            FlightRecorder.write(ch, value)
            return
        }

        if (cachedValues[ch] != value) {
            cachedValues[ch] = value
            FlightRecorder.write(ch, value)
        }
    }



}