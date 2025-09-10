package org.firstinspires.ftc.teamcode.rr.messages

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

class Pose3DMessage(private val pose: Pose3D) {
    @JvmField val timestamp = System.nanoTime()
    @JvmField val x = pose.position.toUnit(DistanceUnit.INCH).x
    @JvmField val y = pose.position.toUnit(DistanceUnit.INCH).y
    @JvmField val heading = pose.orientation.getYaw(AngleUnit.RADIANS)
    @JvmField val position = PositionMessage(pose.position)
    @JvmField
    val orientation = YawPitchRollAnglesMessage(pose.orientation)
}

class PositionMessage(position: Position) {
    @JvmField
    val timestamp = System.nanoTime()
    @JvmField
    val x = position.toUnit(DistanceUnit.INCH).x
    @JvmField
    val y = position.toUnit(DistanceUnit.INCH).y
    @JvmField
    val z = position.toUnit(DistanceUnit.INCH).z
    @JvmField
    val acquisitionTime = position.acquisitionTime
}

class YawPitchRollAnglesMessage(angles: YawPitchRollAngles) {
    @JvmField
    val timestamp = System.nanoTime()
    @JvmField
    val yaw = angles.getYaw(AngleUnit.RADIANS)
    @JvmField
    val pitch = angles.getPitch(AngleUnit.RADIANS)
    @JvmField
    val roll = angles.getRoll(AngleUnit.RADIANS)
    @JvmField
    val acquisitionTime = angles.acquisitionTime
}
