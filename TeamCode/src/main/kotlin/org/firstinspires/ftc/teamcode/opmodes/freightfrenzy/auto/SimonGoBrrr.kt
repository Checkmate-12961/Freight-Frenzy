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
class SimonGoBrrr: BaseOpMode(){

    /**
     * These are the DOs that are allowed in the thing.
     * If it's not in this list, nothing will happen.
     */
    private val legalDos = listOf(
        "MOVE", // Moves the bot by x, y. This is power, not inches!
        "LIFT", // Runs the lift. x = 0: MIN, 1: LOW, 2: MID, 3: HIGH
        "BUCKET", // Makes the bucket dump.
        "CAROUSEL", // Runs the carousel at x power between -1 and 1.
        "TURN" // Turns the robot by x degrees
    )

    // Actions. TODO: Touch.
    companion object {
        // REMEMBER TO LEAVE COMMENTS!
        @JvmField var action0 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action1 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action2 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action3 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action4 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action5 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action6 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action7 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action8 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action9 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action10 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
        //
        @JvmField var action11 = ActionSet(
            "NONE",
            0.0,
            0.0,
            0.0
        )
    }

    // ---------------------------------------- //
    // YOU DON'T NEED ANYTHING BELOW THIS LINE! //
    // ---------------------------------------- //

    // List of actions to take. No touch.
    var actions = mutableListOf<ActionSet>()

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
            action0, action1, action2, action3, action4, action5, action6, action7, action8,
            action9, action10, action11
        )
        for (action in allActions) {
            if (action.DO.uppercase(Locale.ENGLISH) in legalDos) {
                actions += action
            }
        }
    }

    override fun preRunLoop() {
        time = 0.0
    }

    /**
     * No touch.
     */
    override fun runLoop() {
        val action = actions[currentIndex]

        if (currentIndex >= actions.size) {
            requestOpModeStop()
        } else if (!actionIsSet) {
            when(action.DO) {
                "MOVE" -> robot.drivetrain.setWeightedDrivePower(Pose2d(action.x, action.y))
                "LIFT" -> {
                    val liftIndex: Int
                    if (action.x in 0.0..3.0) liftIndex = action.x.toInt()
                    else throw DumbassProgrammerError(
                        "They tried to set an invalid lift position"
                    )
                    robot.lift.target = liftIndices[liftIndex]
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
            time = 0.0
        } else if (time >= action.WAIT) {
            when(action.DO) {
                "MOVE" -> robot.drivetrain.setWeightedDrivePower(Pose2d())
                "BUCKET" -> {
                    robot.bucket.position = Bucket.Positions.REST
                }
                "CAROUSEL" -> {
                    robot.carousel.power = 0.0
                }
            }
            currentIndex += 1
            actionIsSet = false
        }
    }

    // Data class to represent a set of actions. No touch.
    data class ActionSet(@JvmField var DO: String, @JvmField var x: Double, @JvmField var y: Double, @JvmField var WAIT: Double)
}