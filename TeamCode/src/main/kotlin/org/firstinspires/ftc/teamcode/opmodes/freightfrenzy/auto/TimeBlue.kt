package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift
import org.firstinspires.ftc.teamcode.robot.util.DumbassProgrammerError
import java.lang.Math.toRadians
import java.util.*

/**
 * Howdy, Simon! Here's the class where you, yes, *YOU* get to make the robit go brrrrrrrrr.
 * The legalDos list is important. It's all of the actions you can take.
 * If the action only has 1 number, like TURN, LIFT, or CAROUSEL, just use x.
 * If the action has 0 numbers, like BUCKET, ignore both x and y.
 * If you set a WAIT time, the robot will start doing the thing and then go on to the next thing
 *  after the WAIT is over, REGARDLESS OF IF THE THING ACTUALLY FINISHED.
 *
 * @see legalDos for more info.
 */
@Config
@Autonomous(preselectTeleOp = "TeleOp")
class TimeBlue: BaseOpMode(){

    /**
     * These are the DOs that are allowed in the thing.
     * If it's not in this list, nothing will happen.
     */
    private val legalDos = listOf(
        "MOVE", // Moves the bot by x, y. This is power, not inches!
        "LIFT", // Runs the lift. x = 0: MIN, 1: LOW, 2: MID, 3: HIGH, 4: based on the camera value
        "BUCKET", // Makes the bucket dump.
        "CAROUSEL", // Runs the carousel at x power between -1 and 1.
        "TURN" // Turns the robot by x degrees
    )

    // Actions. TODO: Touch.
    // These happen in-order.
    companion object {
        // schmoovin and also to jolt the intake down
        @JvmField var action00 = TimeRed.ActionSet(
            "MOVE",
            .2,
            -1.0,
            0.0
        )
        //turning to align to shipping hub
        @JvmField var action01 = TimeRed.ActionSet(
            "TURN",
            0.0,
            90.0,
            0.0
        )

        //
        @JvmField var action02 = TimeRed.ActionSet(
            "MOVE",
            1.15,
            -.5,
            0.0
        )
            //move forward again
        @JvmField var action03 = TimeRed.ActionSet(
                "TURN",
                0.0,
                -90.0,
                0.0
            )
        //bucket
        @JvmField var action04 = TimeRed.ActionSet(
            "MOVE",
            1.2,
            -.3,
            0.0
        )
        //
        @JvmField var action05 = TimeRed.ActionSet(
            "LIFT",
            1.5,
            4.0,
            0.0
        )
        //
        @JvmField var action06 = TimeRed.ActionSet(
            "BUCKET",
            1.5,
            0.0,
            0.0
        )
        //
        @JvmField var action07 = TimeRed.ActionSet(
            "MOVE",
            .5,
            0.4,
            0.0
        )
        //
        @JvmField var action08 = TimeRed.ActionSet(
            "LIFT",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action09 = TimeRed.ActionSet(
            "TURN",
            0.0,
            90.0,
            0.0
        )
        //
        @JvmField var action10 = TimeRed.ActionSet(
            "MOVE",
            2.0,
            0.0,
            0.0
        )
        //
        @JvmField var action11 = TimeRed.ActionSet(
            "TURN",
            0.0,
            85.0,
            0.0
        )
        //
        @JvmField var action12 = TimeRed.ActionSet(
            "MOVE",
            .4,
            .5,
            0.0
        )
    }

    // ---------------------------------------- //
    // YOU DON'T NEED ANYTHING BELOW THIS LINE! //
    // ---------------------------------------- //

    // List of actions to take. No touch.
    var actions = mutableListOf<TimeRed.ActionSet>()

    private val liftIndices = listOf(
        Lift.Points.MIN,
        Lift.Points.LOW,
        Lift.Points.MID,
        Lift.Points.HIGH
    )

    // Variables that control the flow. No touch.
    private var currentIndex = 0
    private var actionIsSet = false

    /**
     * No touch.
     */
    override fun setup() {
        val allActions = listOf(
            action00, action01, action02, action03, action04, action05, action06, action07,
            action08, action09, action10, action11
        )
        for (action in allActions) {
            if (action.DO.uppercase(Locale.ENGLISH) in legalDos) {
                actions += action
            }
        }
    }

    private var originTimeNano = System.nanoTime()
    private val timeDelta: Double
            get() = (System.nanoTime() - originTimeNano).toDouble() / 1000000000.0


    override fun preRunLoop() {
        originTimeNano = System.nanoTime()
    }

    /**
     * No touch.
     */
    override fun runLoop() {
        // add the current number on the timer to telemetry
        telemetry.addData("Time delta", timeDelta)
        // if there are no more actions to perform, stop the op mode
        if (currentIndex >= actions.size) {
            requestOpModeStop()
            return
        }

        // grab the current action
        val action = actions[currentIndex]

        // if the action is ready to start
        if (!actionIsSet) {
            // do the correct thing
            when(action.DO) {
                "MOVE" -> robot.drivetrain.setWeightedDrivePower(Pose2d(action.x, action.y))
                "LIFT" -> {
                    val liftIndex: Int
                    if (action.x in 0.0..4.0) liftIndex = action.x.toInt()
                    else throw DumbassProgrammerError(
                        "They tried to set an invalid lift position"
                    )
                    if (liftIndex != 4) {
                        robot.lift.target = liftIndices[liftIndex]
                    } else {
                        robot.lift.target = if (liftIndex != 4) {
                            liftIndices[liftIndex]
                        } else {
                            robot.barcode.position.toLiftPoint()
                        }
                    }
                }
                "BUCKET" -> {
                    robot.bucket.position = Bucket.Positions.DUMP
                }
                "CAROUSEL" -> {
                    robot.carousel.power = action.x
                }
                "TURN" -> robot.drivetrain.turn(toRadians(action.x))
            }
            actionIsSet = true
            // reset the timer for the delay
            originTimeNano = System.nanoTime()
        } else if (timeDelta >= action.WAIT) { // otherwise, if the wait is over
            // stop the action
            when(action.DO) {
                "MOVE" -> robot.drivetrain.setWeightedDrivePower(Pose2d())
                "BUCKET" -> {
                    robot.bucket.position = Bucket.Positions.REST
                }
                "CAROUSEL" -> {
                    robot.carousel.power = 0.0
                }
            }
            // bump the action counter
            currentIndex += 1
            actionIsSet = false
        }
    }

    // Data class to represent a set of actions. No touch.
    data class ActionSet(@JvmField var DO: String, @JvmField var WAIT: Double, @JvmField var x: Double, @JvmField var y: Double)
}