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
        .setReversed(true)
        .splineTo(Vector2d(-52.0, 56.0), PI)
        .addDisplacementMarker {
            robot.carousel.power = 1.0
            sleep(1500)
            robot.carousel.power = 0.0
        }
        .setReversed(false)
        .splineTo(Vector2d(-45.0, 54.0), 0.0)
        .turn(-PI * 3/4)
        .splineTo(Vector2d(-58.0, 36.0), -PI/2)
        .addDisplacementMarker{
            println("-" * 10 + "\nRESTART\n" + "-" * 10)
        }
        .build()
}.start()