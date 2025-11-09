package org.firstinspires.ftc.teamcode.helpers.control

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.rr.Localizer

class NullLocalizer: Localizer {
    var pose2  = Pose2d(0.0,0.0,0.0)
    override fun setPose(pose: Pose2d) {
        this.pose2 = pose
    }

    /**
     * Returns the current pose estimate.
     * NOTE: Does not update the pose estimate;
     * you must call update() to update the pose estimate.
     * @return the Localizer's current pose
     */
    override fun getPose(): Pose2d {
       return pose2
    }

    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    override fun update(): PoseVelocity2d {
        return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
    }
}