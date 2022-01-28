package com.checkmaterobotics.meepmeep.util

class Carousel {
    var power = 0.0
        set(value) {
            field = value
            println("Carousel power: $value")
        }
}