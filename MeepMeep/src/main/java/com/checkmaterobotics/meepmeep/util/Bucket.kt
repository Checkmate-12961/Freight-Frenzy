package com.checkmaterobotics.meepmeep.util

class Bucket {
    enum class Positions { DUMP, REST, ZERO }
    var position = Positions.REST
        set(value) {
            field = value
            println("Bucket position: $value")
        }
}