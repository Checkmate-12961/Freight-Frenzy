import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.checkmaterobotics.meepmeep.util.*
import kotlin.math.PI

Config.get().followTrajectorySequence {
    val robot = CheckmateRobot(it)
    robot.barcode.position = Barcode.BarcodePosition.LEFT

    robot.drivetrain.trajectorySequenceBuilder(Pose2d(-34.25, 62.0, PI/2))
        .back(4.0)
        .turn(-PI/2)
        .lineToSplineHeading(Pose2d(-52.0, 56.0, 0.0))
        .addDisplacementMarker {
            robot.carousel.power = 1.0
            sleep(1500)
            robot.carousel.power = 0.0
        }
        .splineTo(Vector2d(-58.0, 36.0), -PI/2)
        .splineTo(Vector2d(-29.0, 25.0), 0.0)
        .addDisplacementMarker{
            robot.lift.target = robot.barcode.position.toLiftPoint()
            sleep(2000)
            robot.bucket.position = Bucket.Positions.DUMP
            sleep(1500)
            robot.lift.target = Lift.Points.MIN
        }
        .lineTo(Vector2d(-40.0, 25.0))
        .splineTo(Vector2d(-58.0, 36.0), PI/2)
        .addDisplacementMarker{
            println("-" * 10 + "\nRESTART\n" + "-" * 10)
        }
        .build()
}.start()