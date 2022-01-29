import com.acmerobotics.roadrunner.geometry.Pose2d
import com.checkmaterobotics.meepmeep.util.Bucket
import com.checkmaterobotics.meepmeep.util.Config
import com.checkmaterobotics.meepmeep.util.Lift
import com.checkmaterobotics.meepmeep.util.sleep
import kotlin.math.PI

Config.start { robot ->
    robot.drivetrain.trajectorySequenceBuilder(Pose2d(-53.0, 61.5, PI/2))
        .back(38.0)
        .turn(PI/2)
        .back(23.5)
        .addDisplacementMarker{
            robot.lift.target = robot.barcode.position.toLiftPoint()
            sleep(1500)
            robot.bucket.position = Bucket.Positions.DUMP
            sleep(2000)
            robot.lift.target = Lift.Points.MIN
        }
        .forward(23.5)
        .turn(-PI/2)
        .forward(12.0)
        .strafeLeft(7.0)
        .build()
}