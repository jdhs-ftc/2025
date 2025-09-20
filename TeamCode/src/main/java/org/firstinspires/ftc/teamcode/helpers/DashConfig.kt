package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import kotlin.reflect.KMutableProperty0

fun <T> registerTunable(prop: KMutableProperty0<T>,category: String)  = FtcDashboard.getInstance().addConfigVariable<T>(category,prop.name,KotlinValueProvider(prop))

class KotlinValueProvider<T>(val prop: KMutableProperty0<T>): ValueProvider<T> {
    override fun get(): T = prop.get()
    override fun set(new: T) = prop.set(new)
}