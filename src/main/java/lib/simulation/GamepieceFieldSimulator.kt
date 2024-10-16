package lib

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class GamepieceFieldSimulator(
    private val field: FRCGameField,
    private val intakePoint: Supplier<Pose2d>,
    private val isIntaking: Supplier<Boolean>,
    private val onIntake: () -> Unit
) {

    private fun render() {
        val displayable =
            field.pieces.filter { it.second }.map { it.first }.toTypedArray()

        Logger.recordOutput("GamepieceVisualizer", *displayable)
    }

    private fun setVisible(index: Int, visible: Boolean) {
        field.pieces[index] = field.pieces[index].copy(second = visible)
    }

    fun reset() {
        field.pieces.forEachIndexed { index, _ -> setVisible(index, true) }
    }

    fun update() {
        field.pieces.forEach { (pose, visible) ->
            if (pose.toPose2d().translation.getDistance(intakePoint.get().translation) < field.gamepieceRadius && isIntaking.get()) {
                setVisible(field.pieces.indexOf(Pair(pose, visible)), false)
                onIntake.invoke()
            }
        }

        render()
    }

    enum class FRCGameField(val pieces: Array<Gamepiece>, val tags: AprilTagFieldLayout, val gamepieceRadius: Double) {
        CRESCENDO(
            arrayOf(
                Gamepiece(Pose3d(2.895854, 4.105616, Units.inchesToMeters(1.0), Rotation3d()), true),
                Gamepiece(Pose3d(2.895854, 4.105616 + 1.4478, Units.inchesToMeters(1.0), Rotation3d()), true),
                Gamepiece(Pose3d(2.895855, 4.105616 + (1.4478 * 2), Units.inchesToMeters(1.0), Rotation3d()), true),
                Gamepiece(Pose3d(8.270526, 0.752816, Units.inchesToMeters(1.0), Rotation3d()), true),
                Gamepiece(Pose3d(8.270526, 0.752816 + 1.6764, Units.inchesToMeters(1.0), Rotation3d()), true),
                Gamepiece(Pose3d(8.270526, 0.752816 + (1.6764 * 2), Units.inchesToMeters(1.0), Rotation3d()), true),
                Gamepiece(Pose3d(8.270526, 0.752816 + (1.6764 * 3), Units.inchesToMeters(1.0), Rotation3d()), true),
                Gamepiece(Pose3d(8.270526, 0.752816 + (1.6764 * 4), Units.inchesToMeters(1.0), Rotation3d()), true),
                Gamepiece(Pose3d(2.895854, 4.105616, Units.inchesToMeters(1.0), Rotation3d()).flip(), true),
                Gamepiece(Pose3d(2.895854, 4.105616 + 1.4478, Units.inchesToMeters(1.0), Rotation3d()).flip(), true),
                Gamepiece(Pose3d(2.895855, 4.105616 + (1.4478 * 2), Units.inchesToMeters(1.0), Rotation3d()).flip(), true),
            ),
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            Units.inchesToMeters(7.0),
        ),
    }
}

typealias Gamepiece = Pair<Pose3d, Boolean>

