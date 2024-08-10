package lib.auto

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
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
 * @param sequentialEventMap Map of path index -> command to run after path finishes, starts at zero
 * @param timeout Timeout in seconds of the routine, defaults to 16, just above the auto period
 */
class ChoreoAuto(
    private val pathGroup: String,
    private val swerve: SwerveSubsystem,
    private val sequentialEventMap: Map<Int, Supplier<Command>>,
    private val parallelEventMap: Map<Int, Supplier<Command>>,
    private val startCommand: Supplier<Command> = Supplier { InstantCommand() },
    private val timeout: Double = 16.0,
    private val shouldReset: Boolean = true,
    ) {
    private val pathList: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup(pathGroup)
//    private var command: Optional<Command> = Optional.empty()
    // Did you forget about null or is there a different reason to use an optional?
    private var command: Command? = null

    /**
     * Factory for creating the actual command group from the event map and path group
     *
     * @param isRed Whether the paths and initial pose need to be flipped or not
     */
    fun createCommand(isRed: Boolean): Command {
        val auto: SequentialCommandGroup = SequentialCommandGroup()
        auto.addCommands(startCommand.get())

        if (shouldReset) {
            if (isRed) {
                auto.addCommands(InstantCommand({ swerve.resetOdometry(pathList.first().flippedInitialPose) }))
            } else {
                auto.addCommands(InstantCommand({ swerve.resetOdometry(pathList.first().initialPose) }))
            }
        }
        pathList.forEachIndexed() { index, path ->
            val parallel: Command = kotlin.collections.Map<Int, Supplier<Command>>::getOrDefault
                .invoke(parallelEventMap, index, Supplier { InstantCommand() }).get()

            auto.addCommands(
                getPath(path, isRed, swerve, parallel),
                kotlin.collections.Map<Int, Supplier<Command>>::getOrDefault
                    .invoke(sequentialEventMap, index, Supplier { InstantCommand() }).get()
            )
        }

        val autoCommand = Commands.waitSeconds(timeout).deadlineWith(auto)
//        command = Optional.of(autoCommand)
        command = autoCommand
        return autoCommand
    }

    /**
     * Cancels the current command of the auto if it is present, else does nothing.
     *
     * @return `true` if the command was canceled, `false` if nothing happened
     */
    fun cancel(): Boolean = command?.cancel() !== null
}