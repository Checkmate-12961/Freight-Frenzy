package org.firstinspires.ftc.teamcode.robot.abstracts

import com.qualcomm.robotcore.util.Range

class Triggerables {
    /**
     * Enum to store the direction the triggerable is triggered in
     */
    enum class TriggerDirection {
        UP, OFF, DOWN;
    }

    /**
     * Interface to manage callbacks for triggerables
     */
    fun interface TriggerableCallback {
        operator fun invoke()
    }

    /**
     * Triggerable class to manage callbacks and states
     */
    abstract class AbstractTriggerable {
        /**
         * Whether or not the triggerable is currently active
         */
        var active: Boolean = false
            protected set

        /**
         * Callback for when the button is pressed
         */
        @JvmField
        var onActivate: TriggerableCallback? = null

        /**
         * Callback for while the button is pressed
         */
        @JvmField
        var whileActive: TriggerableCallback? = null

        /**
         * Callback for when the button is released
         */
        @JvmField
        var onDeactivate: TriggerableCallback? = null

        /**
         * Callback for while the triggerable is deactivated
         */
        @JvmField
        var whileInactive: TriggerableCallback? = null

        /**
         * Updates the triggerable state
         */
        abstract fun update()
    }

    /**
     * Button class to manage callbacks and states
     */
    abstract class AnalogTriggerable : AbstractTriggerable() {
        /**
         * The threshold for triggerable activation
         */
        abstract var activationThreshold: Double
    }

    /**
     * Button class to manage callbacks and states
     */
    class ControllerButton(function: () -> Boolean) : AbstractTriggerable() {
        /**
         * Get the value of the button from the controller
         */
        @JvmField
        val rawPressed: () -> Boolean = function

        /**
         * Updates the button state
         */
        override fun update() {
            // if it's pressed
            if (rawPressed()) {
                // but it doesn't know
                if (!active) {
                    // tell it
                    active = true
                    // then run the activate callback (if it's set)
                    onActivate?.invoke()
                } else {
                    // otherwise run the normal active callback
                    whileActive?.invoke()
                }
            }
            // if it isn't pressed
            else if (!rawPressed()) {
                // but it doesn't know
                if (active) {
                    // tell it
                    active = false
                    // and run the deactivate callback (if it's set)
                    onDeactivate?.invoke()
                } else {
                    // otherwise run the normal inactive callback
                    whileInactive?.invoke()
                }
            }
        }
    }

    /**
     * Trigger class to manage callbacks and states
     */
    class ControllerTrigger(function: () -> Float) : AnalogTriggerable() {
        /**
         * The threshold for triggerable activation
         */
        override var activationThreshold = 0.4

        /**
         * Get the value of the trigger from the controller
         */
        @JvmField
        val rawValue: () -> Float = function

        /**
         * Get corrected value based on the activation threshold
         */
        var correctedValue = 0F
            private set

        /**
         * Updates the trigger state
         */
        override fun update() {
            // update the threshold-corrected value
            correctedValue = Range.scale(rawValue().toDouble(), 0.0, 1.0, activationThreshold, 1.0).toFloat()

            // if it's over the threshold
            if (rawValue() >= activationThreshold) {
                // and it doesn't know it's active
                if (!active) {
                    // set it as active
                    active = true
                    // and run the callback (if it's set)
                    onActivate?.invoke()
                } else {
                    // then run the repeat callback (if it's set)
                    whileActive?.invoke()
                }
            }
            // if it's under the threshold
            else if (rawValue() < activationThreshold) {
                // and it thinks it's active
                if (active) {
                    // set it as inactive
                    active = false
                    // and run the callback (if it's set)
                    onDeactivate?.invoke()
                } else {
                    // then run the repeat callback (if it's set)
                    whileInactive?.invoke()
                }
            }
        }

    }

    /**
     * Stick class to manage callbacks and states
     */
    class ControllerStick(function: () -> Float) : AnalogTriggerable() {
        /**
         * The threshold for stick activation
         */
        override var activationThreshold: Double = 0.3

        /**
         * Get the value of the stick from the controller
         */
        @JvmField
        val rawValue: () -> Float = function

        /**
         * What direction the stick is currently active in
         */
        var state = TriggerDirection.OFF
            private set

        /**
         * What the value of the stick is if it's corrected by the threshold
         */
        var correctedValue = 0.0F
            private set

        /**
         * Callback for when the stick is activated negatively
         */
        @JvmField
        var onActivateNeg: TriggerableCallback? = null

        /**
         * Callback for when the stick is deactivated negatively
         */
        @JvmField
        var onDeactivateNeg: TriggerableCallback? = null

        /**
         * Callback for while the stick is active negatively
         */
        @JvmField
        var whileActiveNeg: TriggerableCallback? = null

        /**
         * Updates the stick state
         */
        override fun update() {
            // update the threshold-corrected value
            correctedValue = if (rawValue() >= 0) {
                Range.scale(rawValue().toDouble(), 0.0, 1.0, activationThreshold, 1.0).toFloat()
            } else {
                -Range.scale(-rawValue().toDouble(), 0.0, 1.0, activationThreshold, 1.0).toFloat()
            }

            when (state) {
                // if it thinks it's up
                TriggerDirection.UP -> {
                    // but it isn't
                    if (rawValue() < activationThreshold) {
                        // tell it
                        active = false
                        state = TriggerDirection.OFF
                        // and run the deactivate callback (if it's set)
                        onDeactivate?.invoke()
                    } else {
                        // otherwise run the normal up callback (if it's set)
                        whileActive?.invoke()
                    }
                }
                // if it thinks it's centered
                TriggerDirection.OFF -> {
                    // but it's up
                    if (rawValue() >= activationThreshold) {
                        // tell it
                        active = true
                        state = TriggerDirection.UP
                        // and run the activate callback (if it's set)
                        onActivate?.invoke()
                    }
                    // but it's down
                    else if (rawValue() <= -activationThreshold) {
                        // tell it
                        active = true
                        state = TriggerDirection.DOWN
                        // and run the negative active callback (if it's set)
                        onActivateNeg?.invoke()
                    } else {
                        // otherwise run the normal off callback (if it's set)
                        whileInactive?.invoke()
                    }
                }
                // if it thinks it's down
                TriggerDirection.DOWN -> {
                    // but it isn't
                    if (rawValue() > -activationThreshold) {
                        // tell it
                        active = false
                        state = TriggerDirection.OFF
                        // and run the negative deactivation callback (if it's set)
                        onDeactivateNeg?.invoke()
                    } else {
                        // otherwise run the normal down callback (if it's set)
                        whileActiveNeg?.invoke()
                    }
                }
            }
        }

    }
}