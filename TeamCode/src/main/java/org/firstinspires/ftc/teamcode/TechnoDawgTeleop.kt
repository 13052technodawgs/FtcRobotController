package org.firstinspires.ftc.teamcode

import android.renderscript.ScriptGroup.Input
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.InputHandler.Button
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sign
import org.firstinspires.ftc.teamcode.InputHandler.GamepadSelection as Gamepad
import org.firstinspires.ftc.teamcode.TechnoDawg.ArmCommand as Command

@TeleOp(name="TechnoDawg Teleop", group="Jacob")
class TechnoDawgTeleop : OpMode() {
    companion object {
        // CONSTANTS
        const val YAW_ADJUST_SPEED = 0.02
        const val ACCELERATION_FORWARD = 0.3
        const val ACCELERATION_RIGHT = 0.3
        const val ACCELERATION_TURN = 0.5
    }

    // HELPERS
    private val robot = TechnoDawg
    private lateinit var input : InputHandler
    private var deltaTime : ElapsedTime = ElapsedTime()

    private var currentSpeedForward : Double = 0.0
    private var currentSpeedRight : Double = 0.0
    private var currentSpeedTurn : Double = 0.0

    override fun init() {
        telemetry.addData("Status", "Initializing")
        telemetry.update()

        input = InputHandler(gamepad1, gamepad2)

        robot.init(this)

        telemetry.addData("Status", "Initialized")

        input.bindCommand(Gamepad.GAMEPAD1, Button.B)       { robot.resetYaw() }
        input.bindCommand(Gamepad.GAMEPAD1, Button.GUIDE)   { robot.launchPlaneIfEndgame() }

        input.bindCommand(Gamepad.GAMEPAD2, Button.GUIDE)   { robot.invokeCommand(Command.STOW) }
        input.bindCommand(Gamepad.GAMEPAD2, Button.A)       { robot.invokeCommand(Command.SCOOP) }
        input.bindCommand(Gamepad.GAMEPAD2, Button.RB)      { robot.invokeCommand(Command.GRAB) }
        input.bindCommand(Gamepad.GAMEPAD2, Button.Y)       { robot.invokeCommand(Command.PLACE) }
        input.bindCommand(Gamepad.GAMEPAD2, Button.DOWN)    { robot.invokeCommand(Command.DROP) }
        input.bindCommand(Gamepad.GAMEPAD2, Button.UP)      { robot.invokeCommand(Command.LIFT) }
    }

    override fun init_loop() {
        if(gamepad1.b) {
            robot.SetTeamColor(true)
        }
        if(gamepad1.x) {
            robot.SetTeamColor(false)
        }
    }

    override fun start() {
        robot.start()
        deltaTime.reset()
    }

    override fun loop() {
        // Get controller inputs

        var forwardTarget = -gamepad1.left_stick_y.toDouble()
        var rightTarget = gamepad1.left_stick_x.toDouble()
        var turnTarget = gamepad2.left_stick_x.toDouble() + gamepad1.right_stick_x.toDouble()
        val deltaYawTrim = YAW_ADJUST_SPEED * (gamepad1.right_trigger - gamepad1.left_trigger)
        input.update()

        // LERP speed

        forwardTarget *= forwardTarget * sign(forwardTarget)
        rightTarget *= rightTarget * sign(rightTarget)
        turnTarget *= turnTarget * sign(turnTarget)

        var forwardDelta = forwardTarget - currentSpeedForward
        var rightDelta = rightTarget - currentSpeedRight
        var turnDelta = turnTarget - currentSpeedTurn

        if(sign(forwardDelta)!=sign(currentSpeedForward)) {
            currentSpeedForward = forwardTarget
        } else {
            currentSpeedForward += sign(forwardDelta) * max(abs(forwardDelta), ACCELERATION_FORWARD * deltaTime.seconds())
        }
        if(sign(rightDelta)!=sign(currentSpeedRight)) {
            currentSpeedRight = rightTarget
        } else {
            currentSpeedRight += sign(rightDelta) * max(abs(rightDelta), ACCELERATION_RIGHT * deltaTime.seconds())
        }

        currentSpeedTurn += sign(turnDelta) * max(abs(turnDelta), ACCELERATION_TURN * deltaTime.seconds())


        robot.trimYaw(deltaYawTrim)
        robot.setDrivePower(currentSpeedRight, currentSpeedForward, currentSpeedTurn)

        robot.update()

        robot.printToTelemetry()
        telemetry.update()

        deltaTime.reset()

    }

    override fun stop() {
        robot.stopAndResetAllEncoders()
    }

}