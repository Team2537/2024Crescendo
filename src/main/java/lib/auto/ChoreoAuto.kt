package lib.auto

import com.choreo.lib.Choreo
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.SwerveSubsystem
import java.util.*
import java.util.function.Supplier

/**
 * Class for defining a choreo autonomous routine with an event map
 *
 * @param pathGroup Name of the path group that is used for this routine
 * @param swerve Swerve subsystem to send control requests to
 * @param shouldReset Boolean to define if the auto routine should reset odometry to the initial position of the first path
 * @param eventMap Map of path index -> command to run after path finishes, starts at zero
 * @param timeout Timeout in seconds of the routine, defaults to 16, just above the auto period
 */
class ChoreoAuto(
    val pathGroup: String,
    val swerve: SwerveSubsystem,
    val shouldReset: Boolean = true,
    val eventMap: Map<Int, Supplier<Command>>,
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
                kotlin.collections.Map<Int, Supplier<Command>>::getOrDefault
                    .invoke(eventMap, index, Supplier { InstantCommand() }).get()
            )
        }

        return Commands.waitSeconds(timeout).deadlineWith(auto)
    }
}