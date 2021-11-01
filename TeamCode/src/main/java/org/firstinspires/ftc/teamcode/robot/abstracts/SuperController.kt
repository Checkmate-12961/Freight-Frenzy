package org.firstinspires.ftc.teamcode.robot.abstracts

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.abs

/**
 * A better controller with extra custom features
 */
@Suppress("unused")
class SuperController(
    /**
     * Internal gamepad
     */
    @JvmField
    val gamepad: Gamepad
){
    /**
     * Button class to manage callbacks and states
     */
    class Button(function: () -> Boolean) {
        /**
         * Get the value of the button from the controller
         */
        val rawPressed: () -> Boolean

        /**
         * Whether or not the button is currently pressed
         */
        var pressed: Boolean = false
            private set

        /**
         * Callback for when the button is pressed
         */
        @JvmField
        var onPress: ButtonCallback? = null

        /**
         * Callback for when the button is released
         */
        @JvmField
        var onRelease: ButtonCallback? = null

        /**
         * Updates the button state
         */
        fun update() {
            if (rawPressed() && !pressed) {
                pressed = true
                onPress?.let { it() }
            } else if (!rawPressed() && pressed) {
                pressed = false
                onRelease?.let { it() }
            }
        }

        init {
            rawPressed = function
        }
    }

    /**
     * Update all buttons
     */
    fun update() {
        leftBumper.update()
        rightBumper.update()
        a.update()
        b.update()
        x.update()
        y.update()
        dpadUp.update()
        dpadDown.update()
        dpadLeft.update()
        dpadRight.update()
        start.update()
        back.update()
        leftStickButton.update()
        rightStickButton.update()
        guide.update()
        share.update()
        options.update()
        touchpad.update()
        cross.update()
        circle.update()
        square.update()
        triangle.update()
        ps.update()
    }

    /**
     * Minimum dead zone threshold
     */
    private val deadZoneMin = 0.2F

    /**
     * left analog stick horizontal axis
     */
    val leftStickX: Float
        get() {
            val value = gamepad.left_stick_x
            return if (abs(value) > deadZoneMin) {value}
            else {0F}
        }
    /**
     * left analog stick vertical axis
     */
    val leftStickY: Float
        get() {
            val value = gamepad.left_stick_y
            return if (abs(value) > deadZoneMin) {value}
            else {0F}
        }
    /**
     * right analog stick horizontal axis
     */
    val rightStickX: Float
        get() {
            val value = gamepad.right_stick_x
            return if (abs(value) > deadZoneMin) {value}
            else {0F}
        }
    /**
     * right analog stick vertical axis
     */
    val rightStickY: Float
        get() {
            val value = gamepad.right_stick_y
            return if (abs(value) > deadZoneMin) {value}
            else {0F}
        }

    /**
     * dpad up
     */
    @JvmField
    val dpadUp = Button { gamepad.dpad_up }

    /**
     * dpad down
     */
    @JvmField
    val dpadDown = Button { gamepad.dpad_down }

    /**
     * dpad left
     */
    @JvmField
    val dpadLeft = Button { gamepad.dpad_left }

    /**
     * dpad right
     */
    @JvmField
    val dpadRight = Button { gamepad.dpad_right }

    /**
     * button a
     */
    @JvmField
    val a = Button { gamepad.a }

    /**
     * button b
     */
    @JvmField
    val b = Button { gamepad.b }

    /**
     * button x
     */
    @JvmField
    val x = Button { gamepad.x }

    /**
     * button y
     */
    @JvmField
    val y = Button { gamepad.y }

    /**
     * button guide - often the large button in the middle of the controller. The OS may capture this button before it is sent to the app; in which case you'll never receive it.
     */
    @JvmField
    val guide = Button { gamepad.guide }

    /**
     * button start
     */
    @JvmField
    val start = Button { gamepad.start }

    /**
     * button back
     */
    @JvmField
    val back = Button { gamepad.back }

    /**
     * button left bumper
     */
    @JvmField
    val leftBumper = Button { gamepad.left_bumper }

    /**
     * button right bumper
     */
    @JvmField
    val rightBumper = Button { gamepad.right_bumper }

    /**
     * left stick button
     */
    @JvmField
    val leftStickButton = Button { gamepad.left_stick_button }

    /**
     * right stick button
     */
    @JvmField
    val rightStickButton = Button { gamepad.right_stick_button }

    /**
     * left trigger
     */
    val leftTrigger: Float
        get() {
            return gamepad.left_trigger
        }

    /**
     * right trigger
     */
    val rightTrigger: Float
        get() {
            return gamepad.right_trigger
        }

    /**
     * PS4 Support - Circle
     */
    @JvmField
    val circle = Button { gamepad.circle }

    /**
     * PS4 Support - Cross
     */
    @JvmField
    val cross = Button { gamepad.cross }

    /**
     * PS4 Support - Triangle
     */
    @JvmField
    val triangle = Button { gamepad.triangle }

    /**
     * PS4 Support - Square
     */
    @JvmField
    val square = Button { gamepad.square }

    /**
     * PS4 Support - Share
     */
    @JvmField
    val share = Button { gamepad.share }

    /**
     * PS4 Support - Options
     */
    @JvmField
    val options = Button { gamepad.options }

    /**
     * PS4 Support - touchpad
     */
    @JvmField
    val touchpad = Button { gamepad.touchpad }
    val touchpadFinger1: Vector2d?
        get() {
            return if (gamepad.touchpad_finger_1){
                Vector2d(gamepad.touchpad_finger_1_x.toDouble(),
                    gamepad.touchpad_finger_1_y.toDouble()
                )
            } else {null}
        }
    val touchpadFinger2: Vector2d?
        get() {
            return if (gamepad.touchpad_finger_2){
                Vector2d(gamepad.touchpad_finger_2_x.toDouble(),
                    gamepad.touchpad_finger_2_y.toDouble()
                )
            } else {null}
        }

    /**
     * PS4 Support - PS Button
     */
    @JvmField
    val ps = Button { gamepad.ps }
}