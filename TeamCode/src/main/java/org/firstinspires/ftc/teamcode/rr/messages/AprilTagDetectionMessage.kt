package org.firstinspires.ftc.teamcode.rr.messages

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

class AprilTagDetectionMessage(private val detection: AprilTagDetection) {
    @JvmField val id = detection.id
    @JvmField val hamming = detection.hamming
    @JvmField val decisionMargin = detection.decisionMargin.toDouble()
    //@JvmField val center = detection.center
    //@JvmField val corners = detection.corners
    //val metadata = detection.metadata
    //@JvmField val ftcPose = detection.ftcPose
    //@JvmField val rawPose = detection.rawPose
    @JvmField val robotPose = Pose3DMessage(detection.robotPose)
    @JvmField val frameAcquisitionNanoTime = detection.frameAcquisitionNanoTime
    @JvmField val currentTimeActual = System.nanoTime()
    @JvmField val lagNanos = currentTimeActual - frameAcquisitionNanoTime
}