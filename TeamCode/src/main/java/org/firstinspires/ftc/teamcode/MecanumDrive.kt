package org.firstinspires.ftc.teamcode

import kotlin.math.cos
import kotlin.math.sin

class MecanumDrive {
    private var power = arrayOf(0.0, 0.0, 0.0, 0.0)

    val powerFrontRight: Double get() = power[0]
    val powerFrontLeft: Double get() = power[1]
    val powerRearLeft: Double get() = power[2]
    val powerRearRight: Double get() = power[3]

    private var forceFrontRight = Vector2.copy(FORCE_EVEN)
    private var forceFrontLeft = -Vector2.copy(FORCE_ODD)

    val totalYaw: Double get() = yaw+yawTrim
    var yaw: Double = 0.0
        set(newYaw) {
            val deltaYaw = newYaw - yaw
            // Counter-rotate the forces to account for robot yaw
            forceFrontRight.rotate(-deltaYaw)
            forceFrontLeft.rotate(-deltaYaw)
            field = newYaw
        }

    var yawTrim: Double = 0.0
        set(newYawTrim) {
            val deltaYawTrim = newYawTrim - yawTrim
            // Counter-rotate the forces to account for robot yaw
            forceFrontRight.rotate(-deltaYawTrim)
            forceFrontLeft.rotate(-deltaYawTrim)
            field = newYawTrim
        }

    fun calculateDrivePower(x: Double, y: Double, r: Double) {
        val translate = Vector2(x, y)
        calculateDrivePower(translate, r)
    }

    fun calculateDrivePower(translate: Vector2, rotate: Double) {
        power[0] =   forceFrontRight.dot(translate) - rotate/2.0
        power[1] =    forceFrontLeft.dot(translate) - rotate/2.0
        power[2] = - forceFrontRight.dot(translate) - rotate/2.0
        power[3] = -  forceFrontLeft.dot(translate) - rotate/2.0
    }

    fun resetYaw() {
        yaw = 0.0
        yawTrim = 0.0

        forceFrontRight = Vector2.copy(FORCE_EVEN)
        forceFrontLeft = -Vector2.copy(FORCE_ODD)
    }

    fun stop() {
        power[0]=0.0
        power[1]=0.0
        power[2]=0.0
        power[3]=0.0
    }
    companion object {
        val FORCE_EVEN = Vector2(cos(Math.PI * 3/4), sin(Math.PI * 3/4))
        val FORCE_ODD  = Vector2(cos(Math.PI * 1/4), sin(Math.PI * 1/4))
    }
}
