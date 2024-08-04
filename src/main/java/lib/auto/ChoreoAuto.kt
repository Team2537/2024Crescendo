package lib.auto

import com.choreo.lib.Choreo
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.SwerveSubsystem
import java.util.*
import kotlin.jvm.optionals.getOrDefault

class ChoreoAuto(
    val pathGroup: String,
    val swerve: SwerveSubsystem,
    val shouldReset: Boolean = false,
    val eventMap: Map<Int, Command>,
    val timeout: Double = 16.0
) {
    val pathList = Choreo.getTrajectoryGroup(pathGroup)

    fun createCommand(isRed: Boolean): Command {
        val auto: SequentialCommandGroup = SequentialCommandGroup()
        if (shouldReset) {
            if (isRed) {
                auto.addCommands(InstantCommand({ swerve.resetOdometry(pathList.first().flippedInitialPose) }))
            } else {
                auto.addCommands(InstantCommand({ swerve.resetOdometry(pathList.first().initialPose) }))
            }
        }
        pathList.forEachIndexed() { index, path ->
            auto.addCommands(
                getPath(path, isRed, swerve),
                eventMap.getOrDefault(index, InstantCommand())
            )
        }

        return auto
    }
}