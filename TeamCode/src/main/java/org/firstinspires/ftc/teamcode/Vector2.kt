package org.firstinspires.ftc.teamcode

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Vector2(var x: Double, var y: Double) {
    val magnitude: Double
        get() = sqrt(x*x+y*y)

    fun normalize() {
        val m = magnitude
        x /= m
        y /= m
    }

    operator fun unaryMinus() = Vector2(-x, -y)
    operator fun plus(other: Vector2) = Vector2(x+other.x, y+other.y)
    operator fun minus(other: Vector2) = Vector2(x-other.x, y-other.y)
    operator fun times(scalar: Double) = Vector2(x*scalar, y*scalar)

    fun dot(other: Vector2): Double {
        return x * other.x + y * other.y
    }
    fun rotate(radians: Double) {
        val newX = cos(radians) * x - sin(radians) * y
        val newY = sin(radians) * x + cos(radians) * y
        x = newX
        y = newY
    }

    companion object {
        fun copy(original: Vector2): Vector2 {
            return Vector2(original.x, original.y)
        }

        fun unitVector(radians: Double): Vector2 {
            return Vector2(cos(radians), sin(radians))
        }
    }

}