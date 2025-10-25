package org.firstinspires.ftc.teamcode.helpers

fun Map<Double,Double>.interp(x : Double): Double {
    val prev = this.filterKeys { it < x }.maxByOrNull { it.key } ?: return this.values.first()
    val next = this.filterKeys { it > x }.minByOrNull { it.key } ?: return this.values.last()

    val rel = (x - prev.key) / (next.key - prev.key)
    return prev.value + (next.value - prev.value) * rel
}