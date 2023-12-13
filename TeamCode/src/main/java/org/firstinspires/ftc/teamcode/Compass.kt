package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.PI
import kotlin.math.abs

class Compass {
    companion object {
        // PID coefficients
        const val kP = 0.4
        const val kI = 0.00108
        const val kD = 0.0

        const val COMPASS_ROTATIONS_PER_TICK = 3.0/560.0
    }

    // PID terms
    private var p = 0.0
    private var i = 0.0
    private var d = 0.0

    // Encoder values
    var ticks = 0
        private set
    var angle = 0.0
        private set(angle) {
            // Constrain angle between 0 and 2PI
            field = angle % (2*PI)
            if(field < 0) field += (2*PI)
        }
    var target = 0.0
        set(target) {
            // Constrain target to between 0 and 2PI
            field = target % (2*PI)
            if (field < 0) field += 2*PI
        }

    // Other factors
    private var lastError = 0.0

    private val deltaTime = ElapsedTime()
    private var firstFrame = true

    val idealPower get() = -(p+i+d)

    fun update(ticks: Int) {
        this.ticks = ticks
        angle = ticks * 2.0 * COMPASS_ROTATIONS_PER_TICK * PI

        // If the error > PI, go the other way (it's shorter)
        var angleError = target - angle
        if(angleError > PI) angleError -= (2*PI)

        if(firstFrame) deltaTime.reset()

        if(abs(angleError) < 0.001) angleError = 0.0

        p  = kP * angleError
        i += kI * lastError * deltaTime.milliseconds()
        d = if(deltaTime.milliseconds() > 0)
            kD * (angleError - lastError) / deltaTime.milliseconds()
        else
            0.0

        lastError = angleError
        deltaTime.reset()
    }



}