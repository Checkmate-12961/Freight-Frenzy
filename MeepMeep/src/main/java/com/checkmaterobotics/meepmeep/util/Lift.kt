package com.checkmaterobotics.meepmeep.util

class Lift {
    /**
     * Represents the possible positions of the lift.
     */
    enum class Points {
        LOW, MID, HIGH, MIN, MAX
    }
    /**
     * Whether the lift is at the bottom and not making corrections.
     */
    var isDown: Boolean = true
        private set

    var target: Points = Points.MIN
        set(value) {
            field = value
            isDown = value == Points.MIN
            println("Lift target: $value")
        }
}