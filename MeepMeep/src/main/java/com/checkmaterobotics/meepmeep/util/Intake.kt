package com.checkmaterobotics.meepmeep.util

class Intake {
    var power = 0.0
        set(value) {
            field = value
            println("Intake power: $value")
        }
}