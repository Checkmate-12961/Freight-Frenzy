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
    val dpadUp: Boolean
        get() {
            return gamepad.dpad_up
        }

    /**
     * dpad down
     */
    val dpadDown: Boolean
        get() {
            return gamepad.dpad_down
        }

    /**
     * dpad left
     */
    val dpadLeft: Boolean
        get() {
            return gamepad.dpad_left
        }

    /**
     * dpad right
     */
    val dpadRight: Boolean
        get() {
            return gamepad.dpad_right
        }

    /**
     * button a
     */
    val a: Boolean
        get() {
            return gamepad.a
        }

    /**
     * button b
     */
    val b: Boolean
        get() {
            return gamepad.b
        }

    /**
     * button x
     */
    val x: Boolean
        get() {
            return gamepad.x
        }

    /**
     * button y
     */
    val y: Boolean
        get() {
            return gamepad.y
        }

    /**
     * button guide - often the large button in the middle of the controller. The OS may capture this button before it is sent to the app; in which case you'll never receive it.
     */
    val guide: Boolean
        get() {
            return gamepad.guide
        }

    /**
     * button start
     */
    val start: Boolean
        get() {
            return gamepad.start
        }

    /**
     * button back
     */
    val back: Boolean
        get() {
            return gamepad.back
        }

    /**
     * button left bumper
     */
    val leftBumper: Boolean
        get() {
            return gamepad.left_bumper
        }

    /**
     * button right bumper
     */
    val rightBumper: Boolean
        get() {
            return gamepad.right_bumper
        }

    /**
     * left stick button
     */
    val leftStickButton: Boolean
        get() {
            return gamepad.left_stick_button
        }

    /**
     * right stick button
     */
    val rightStickButton: Boolean
        get() {
            return gamepad.right_stick_button
        }

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
    val circle: Boolean
        get () {
            return gamepad.circle
        }

    /**
     * PS4 Support - Cross
     */
    val cross: Boolean
        get () {
            return gamepad.cross
        }

    /**
     * PS4 Support - Triangle
     */
    val triangle: Boolean
        get () {
            return gamepad.triangle
        }

    /**
     * PS4 Support - Square
     */
    val square: Boolean
        get () {
            return gamepad.square
        }

    /**
     * PS4 Support - Share
     */
    val share: Boolean
        get () {
            return gamepad.share
        }

    /**
     * PS4 Support - Options
     */
    val options: Boolean
        get () {
            return gamepad.options
        }

    /**
     * PS4 Support - touchpad
     */
    val touchpad: Boolean
        get() {
            return gamepad.touchpad
        }
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
    val ps: Boolean
        get() {
            return gamepad.ps
        }
}