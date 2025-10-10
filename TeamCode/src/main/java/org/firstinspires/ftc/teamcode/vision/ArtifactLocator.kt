package org.firstinspires.ftc.teamcode.vision

import android.graphics.Color.rgb
import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.helpers.registerTunable
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion


class ArtifactLocator(hardwareMap: HardwareMap) {
    /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for. Use a predefined color, or create your own
         *
         *   .setTargetColorRange(ColorRange.BLUE)     // use a predefined color match
         *     Available colors are: RED, BLUE, YELLOW, GREEN, ARTIFACT_GREEN, ARTIFACT_PURPLE
         *   .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,  // or define your own color match
         *                                       new Scalar( 32, 176,  0),
         *                                       new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *       ImageRegion.entireFrame()
         *       ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixels at upper left corner
         *       ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height in center
         *
         * - Define which contours are included.
         *   You can get ALL the contours, ignore contours that are completely inside another contour.
         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
         *     EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up solid colors.
         *
         * - Turn the displays of contours ON or OFF.
         *     Turning these on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)                Draws an outline of each contour.
         *        .setEnclosingCircleColor(int color)   Draws a circle around each contour. 0 to disable.
         *        .setBoxFitColor(int color)            Draws a rectangle around each contour. 0 to disable. ON by default.
         *
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.
         *     Using these features requires an understanding of how they may effect the final
         *     blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)
         *        Blurring an image helps to provide a smooth color transition between objects,
         *        and smoother contours.  The higher the number, the more blurred the image becomes.
         *        Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *        Blurring too much may hide smaller features.  A size of 5 is good for a 320x240 image.
         *
         *     .setErodeSize(int pixels)
         *        Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *        Erosion can grow holes inside regions, and also shrink objects.
         *        "pixels" in the range of 2-4 are suitable for low res images.
         *
         *     .setDilateSize(int pixels)
         *        Dilation makes objects and lines more visible by filling in small holes, and making
         *        filled shapes appear larger. Dilation is useful for joining broken parts of an
         *        object, such as when removing noise from an image.
         *        "pixels" in the range of 2-4 are suitable for low res images.
         *
         *        .setMorphOperationType(MorphOperationType morphOperationType)
         *        This defines the order in which the Erode/Dilate actions are performed.
         *        OPENING:    Will Erode and then Dilate which will make small noise blobs go away
         *        CLOSING:    Will Dilate and then Erode which will tend to fill in any small holes in blob edges.
         */
    var purpleLocator: ColorBlobLocatorProcessor = ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // Use a predefined color match
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
        .setDrawContours(true) // Show contours on the Stream Preview
        .setBoxFitColor(0) // Disable the drawing of rectangles
        .setCircleFitColor(rgb(255, 0, 255)) // Draw a circle
        .setBlurSize(5) // Smooth the transitions between different colors in image
        // the following options have been added to fill in perimeter holes.

        .setDilateSize(15) // Expand blobs to fill any divots on the edges
        .setErodeSize(15) // Shrink blobs back to original size
        .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

        .build()

    var greenLocator: ColorBlobLocatorProcessor = ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ColorRange.ARTIFACT_GREEN) // Use a predefined color match
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
        .setDrawContours(true) // Show contours on the Stream Preview
        .setBoxFitColor(0) // Disable the drawing of rectangles
        .setCircleFitColor(rgb(0, 255, 0)) // Draw a circle
        .setBlurSize(5) // Smooth the transitions between different colors in image
        // the following options have been added to fill in perimeter holes.

        .setDilateSize(15) // Expand blobs to fill any divots on the edges
        .setErodeSize(15) // Shrink blobs back to original size
        .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

        .build()

    /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution
         *      that is supported by your camera.  This will improve overall performance and reduce
         *      latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
    var portal: VisionPortal = VisionPortal.Builder()
        .addProcessors(purpleLocator, greenLocator)
        .setCameraResolution(Size(320, 240))
        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 2"))
        .enableLiveView(false)
        .build()

    fun getFilteredBlobs(color: Color = Color.ANY): List<ColorBlobLocatorProcessor.Blob> {

        // Read the current list
        val blobs = if (color == Color.PURPLE) {
            purpleLocator.blobs
        } else if (color == Color.GREEN) {
            greenLocator.blobs
        } else {
            purpleLocator.blobs + greenLocator.blobs
        }



        /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those
             *          that satisfy the filter conditions will remain in the current list of
             *          "blobs".  Multiple filters may be used.
             *
             * To perform a filter
             *   ColorBlobLocatorProcessor.Util.filterByCriteria(criteria, minValue, maxValue, blobs);
             *
             * The following criteria are currently supported.
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any
             *   that are too big or small. Start with a large range and then refine the range based
             *   on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the
             *   contour. The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_ARC_LENGTH
             *   A blob's arc length is the perimeter of the blob.
             *   This can be used in conjunction with an area filter to detect oddly shaped blobs.
             *
             * ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY
             *   A blob's circularity is how circular it is based on the known area and arc length.
             *   A perfect circle has a circularity of 1.  All others are < 1
             */
        ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
            50.0, 20000.0, blobs
        ) // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
            0.6, 1.0, blobs
        ) /* filter out non-circular blobs.
                    * NOTE: You may want to adjust the minimum value depending on your use case.
                    * Circularity values will be affected by shadows, and will therefore vary based
                    * on the location of the camera on your robot and venue lighting. It is strongly
                    * encouraged to test your vision on the competition field if your event allows
                    * sensor calibration time.
                    */



        return blobs.sortedBy { it.contourArea }.reversed()
    }

    companion object {
        var p = 0.01
        var i = 0.0
        var d = 0.0
        var kV = 0.0
        var kA = 0.0
        var kStatic = 0.0

        init {
            registerTunable(::p, "ArtifactLocator")
            registerTunable(::i, "ArtifactLocator")
            registerTunable(::d, "ArtifactLocator")
            registerTunable(::kV, "ArtifactLocator")
            registerTunable(::kA, "ArtifactLocator")
            registerTunable(::kStatic, "ArtifactLocator")
        }
    }

    val pid = PIDFController.PIDCoefficients(p, i, d)
    val controller = PIDFController(pid, kV, kA, kStatic)

    var blobs: List<ColorBlobLocatorProcessor.Blob> = listOf()
    var currentPos = 0.0
    var targetPos = 120.0

    init {
        controller.targetPosition = targetPos
    }

    fun correct(color: Color = Color.ANY): Double {
        blobs = getFilteredBlobs(color)
        if (blobs.isEmpty()) return 0.0
        currentPos = blobs[0].circle.center.x
        return controller.update(blobs[0].circle.center.x)

    }
}