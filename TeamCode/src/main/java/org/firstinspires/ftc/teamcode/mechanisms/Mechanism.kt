package org.firstinspires.ftc.teamcode.mechanisms

import org.firstinspires.ftc.teamcode.helpers.LogTelemetry

fun interface Mechanism {
    fun update(telemetry: LogTelemetry)
}