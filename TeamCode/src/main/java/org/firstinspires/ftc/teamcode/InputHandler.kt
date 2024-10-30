package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad

class InputHandler (gamepad1: Gamepad, gamepad2: Gamepad) {
    private var gamepads = mutableMapOf(
        GamepadSelection.GAMEPAD1 to gamepad1,
        GamepadSelection.GAMEPAD2 to gamepad2
    )

    enum class Button {
        A, B, X, Y, UP, DOWN, LEFT, RIGHT, LB, RB, BACK, START, GUIDE, LSB, RSB
    }

    enum class GamepadSelection {
        GAMEPAD1, GAMEPAD2
    }

    private var buttonBindings = GamepadSelection.values().associateWithTo(mutableMapOf()) {
        Button.values().associateWithTo(mutableMapOf()) {
            mutableSetOf<()->Unit>()
        }
    }

    var buttonState = GamepadSelection.values().associateWithTo(mutableMapOf()) {
        Button.values().associateWithTo(mutableMapOf()) { false }
    }
        private set

    private var lastButtonState = GamepadSelection.values().associateWithTo(mutableMapOf()) {
        Button.values().associateWithTo(mutableMapOf()) { false }
    }

    fun update() {
        for(gamepad in GamepadSelection.values()) {
            buttonState.getValue(gamepad)[Button.A] = gamepads.getValue(gamepad).a
            buttonState.getValue(gamepad)[Button.B] = gamepads.getValue(gamepad).b
            buttonState.getValue(gamepad)[Button.X] = gamepads.getValue(gamepad).x
            buttonState.getValue(gamepad)[Button.Y] = gamepads.getValue(gamepad).y
            buttonState.getValue(gamepad)[Button.UP] = gamepads.getValue(gamepad).dpad_up
            buttonState.getValue(gamepad)[Button.DOWN] = gamepads.getValue(gamepad).dpad_down
            buttonState.getValue(gamepad)[Button.LEFT] = gamepads.getValue(gamepad).dpad_left
            buttonState.getValue(gamepad)[Button.RIGHT] = gamepads.getValue(gamepad).dpad_right
            buttonState.getValue(gamepad)[Button.LB] = gamepads.getValue(gamepad).left_bumper
            buttonState.getValue(gamepad)[Button.RB] = gamepads.getValue(gamepad).right_bumper
            buttonState.getValue(gamepad)[Button.BACK] = gamepads.getValue(gamepad).back
            buttonState.getValue(gamepad)[Button.START] = gamepads.getValue(gamepad).start
            buttonState.getValue(gamepad)[Button.GUIDE] = gamepads.getValue(gamepad).guide
            buttonState.getValue(gamepad)[Button.LSB] = gamepads.getValue(gamepad).left_stick_button
            buttonState.getValue(gamepad)[Button.RSB] = gamepads.getValue(gamepad).right_stick_button

            for(button in Button.values()) {
                // If the button has been pressed and wasn't previously pressed
                if(buttonState.getValue(gamepad).getValue(button) &&
                    !lastButtonState.getValue(gamepad).getValue(button)) {
                    for(command in buttonBindings.getValue(gamepad).getValue(button))
                        command()
                }
            }
        }

        // Update last state to current values
        for(gamepad in GamepadSelection.values()) {
            for (button in Button.values()) {
                lastButtonState.getValue(gamepad)[button] =
                    buttonState.getValue(gamepad).getValue(button)
            }
        }
    }

    fun bindCommand(gamepad: GamepadSelection, button: Button, command: ()->Unit) {
        buttonBindings.getValue(gamepad).getValue(button).add(command)
    }


}