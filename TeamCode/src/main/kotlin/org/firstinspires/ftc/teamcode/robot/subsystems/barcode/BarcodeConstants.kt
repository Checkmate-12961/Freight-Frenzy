package org.firstinspires.ftc.teamcode.robot.subsystems.barcode

import org.opencv.core.Point
import org.opencv.core.Rect

object BarcodeConstants {
    @JvmField var leftBox = DetectionBox(
        225,
        145,
        50,
        50,
        125
    )
    @JvmField var middleBox = DetectionBox(
        20,
        145,
        50,
        50,
        125
    )
}

class DetectionBox(
    @JvmField var x: Int,
    @JvmField var y: Int,
    @JvmField var width: Int,
    @JvmField var height: Int,
    @JvmField var threshold: Int
) {
    val pointA: Point
        get() = Point(x.toDouble(), y.toDouble())

    val pointB: Point
        get() = Point((x + width).toDouble(), (y + height).toDouble())

    val rectangle: Rect
        get() = Rect(pointA, pointB)
}