package org.firstinspires.ftc.teamcode.rr.messages

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.opencv.core.Point

class AprilTagDetectionMessage(private val detection: AprilTagDetection) {
    @JvmField val id = detection.id
    @JvmField val hamming = detection.hamming
    @JvmField val decisionMargin = detection.decisionMargin.toDouble()
    @JvmField val center = PointMessage(detection.center)
    @JvmField val corners = detection.corners.map { PointMessage(it) }
    val metadata = detection.metadata
    @JvmField val ftcPose = detection.ftcPose
    //@JvmField val rawPose = detection.rawPose
    @JvmField val robotPose = Pose3DMessage(detection.robotPose)
    @JvmField val frameAcquisitionNanoTime = detection.frameAcquisitionNanoTime
    @JvmField val currentTimeActual = System.nanoTime()
    @JvmField val lagNanos = currentTimeActual - frameAcquisitionNanoTime
}

class PointMessage(private val point: Point) {
    @JvmField
    val x = point.x
    @JvmField
    val y = point.y
}