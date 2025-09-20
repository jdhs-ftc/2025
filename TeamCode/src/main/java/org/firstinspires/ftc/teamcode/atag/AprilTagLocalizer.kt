package org.firstinspires.ftc.teamcode.atag

import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.helpers.vision.CameraStreamProcessor
import org.firstinspires.ftc.teamcode.rr.Localizer
import org.firstinspires.ftc.teamcode.rr.messages.AprilTagDetectionMessage
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Math.toDegrees
import kotlin.jvm.java


class AprilTagLocalizer(val hardwareMap: HardwareMap, val baseLocalizer: Localizer): Localizer by baseLocalizer {

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private val cameraPosition = Position(
        DistanceUnit.INCH,
        0.0, 0.0, 0.0, 0
    )
    private val cameraOrientation = YawPitchRollAngles(
        AngleUnit.DEGREES,
        -90.0, -90.0, 0.0, 0
    )

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private val aprilTag = AprilTagProcessor.Builder()

    // The following default settings are available to un-comment and edit as needed.
    //.setDrawAxes(false)
    //.setDrawCubeProjection(false)
    //.setDrawTagOutline(true)
    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
    .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
    .setCameraPose(cameraPosition, cameraOrientation)

    // == CAMERA CALIBRATION ==
    // If you do not manually specify calibration parameters, the SDK will attempt
    // to load a predefined calibration for your camera.
    // 480p veer calib .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
     // 480p my calib
    .setLensIntrinsics(516.3798424, 515.8231389, 328.1776587, 237.3745503 )
    // 240P .setLensIntrinsics(281.5573273, 281.366942, 156.3332591, 119.8965271)
    // ... these parameters are fx, fy, cx, cy.


    .build();

    /**
     * The variable to store our instance of the vision portal.
     */
    private val visionPortal: VisionPortal


    init {
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1.0F);

        // Create the vision portal by using a builder.
        val builder = VisionPortal.Builder()


        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))



        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(Size(640, 480))


        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)


        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag)
        builder.addProcessor(CameraStreamProcessor())


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build()
    }


    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    override fun update(): PoseVelocity2d {
        val vel = baseLocalizer.update()
        //return vel
        /*
        if (vel.linearVel.norm().toDouble() > 1.0 || toDegrees(vel.angVel.toDouble()) > 1.0) {
            return vel
        }

         */

        val currentDetections = aprilTag.freshDetections ?: return vel

        // Step through the list of detections and display info for each one.
        val foundPoses: ArrayList<Pose2d> = arrayListOf()
        for (detection in currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    FlightRecorder.write("AprilTagLocalizer/detection${detection.id}", AprilTagDetectionMessage(detection))
                    foundPoses.add(Pose2d(detection.robotPose.position.x, detection.robotPose.position.y, detection.robotPose.orientation.getYaw(AngleUnit.RADIANS)))
                }
            }
        } // end for() loop

        foundPoses.sortBy { (it - pose).line.norm() }

        pose = foundPoses.firstOrNull() ?: return vel



        return vel
    }
}