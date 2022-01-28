package com.checkmaterobotics.meepmeep.util

class Barcode {
    /**
     * Represents the position of the TSE on the barcode.
     */
    enum class BarcodePosition {
        LEFT, MIDDLE, RIGHT;

        /**
         * Get the corresponding [Lift.Points] entry.
         *
         * @return The corresponding [Lift.Points] entry.
         */
        fun toLiftPoint(): Lift.Points {
            return when(this) {
                LEFT -> Lift.Points.HIGH
                MIDDLE -> Lift.Points.MID
                RIGHT -> Lift.Points.LOW
            }
        }
    }

    var position = BarcodePosition.LEFT
}