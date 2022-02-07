import com.acmerobotics.roadrunner.geometry.Pose2d
import com.checkmaterobotics.meepmeep.util.Bucket
import com.checkmaterobotics.meepmeep.util.Config
import com.checkmaterobotics.meepmeep.util.Lift
import com.checkmaterobotics.meepmeep.util.sleep
import kotlin.math.PI

Config.start { robot ->
    robot.drivetrain.trajectorySequenceBuilder(Pose2d(-54.0, -61.5, -PI /2))
        .back(15.0)
        .strafeLeft(42.5)
        .back(5.0)
        .addDisplacementMarker{
            robot.lift.target = Lift.Points.HIGH
            sleep(2000)
            robot.bucket.position = Bucket.Positions.DUMP
            sleep(3500)
            robot.lift.target = Lift.Points.MIN
        }
        .forward(5.0)
        .strafeRight(42.5)
        .back(11.0)
        .strafeRight(8.0)
        .build()
}