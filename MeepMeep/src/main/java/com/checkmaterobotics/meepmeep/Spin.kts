import com.acmerobotics.roadrunner.geometry.Pose2d
import com.checkmaterobotics.meepmeep.util.CheckmateRobot
import com.checkmaterobotics.meepmeep.util.Config
import kotlin.math.PI

Config.get().followTrajectorySequence {
    val robot = CheckmateRobot(it)

    robot.drivetrain.trajectorySequenceBuilder(Pose2d())
        .turn(2 * PI)
        .build()
}.start()