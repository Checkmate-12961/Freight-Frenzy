package com.checkmaterobotics.meepmeep.util

import com.noahbres.meepmeep.roadrunner.DriveShim
import kotlin.math.floor

fun sleep(ms: Long) {
    println("Wait $ms milliseconds")
}

class CheckmateRobot(val drivetrain: DriveShim) {
    val carousel = Carousel()
    val lift = Lift()
    val intake = Intake()
    val bucket = Bucket()
    val barcode = Barcode()

    init {
        println("-" * 10 + "\nRESTART\n" + "-" * 10)
    }
}

operator fun String.times(amount: Int): String {
    var temp = ""
    for (i in 0..amount) {
        temp += this
    }
    return temp
}
