package frc.robot.commands

import LauncherSubsystem
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants
import frc.robot.commands.launcher.FireCommand
import frc.robot.commands.launcher.PrimeLauncherCommand
import frc.robot.commands.launcher.ReadyFireCommand
import frc.robot.commands.pivot.QuickPivotCommand
import frc.robot.subsystems.SwerveSubsystem
import java.util.function.Supplier

object Autos {
    private val autoModeChooser =
        SendableChooser<AutoMode>().apply {
            AutoMode.entries.forEach { addOption(it.optionName, it) }
            setDefaultOption(AutoMode.default.optionName, AutoMode.default)
        }

    val defaultAutonomousCommand: Command
        get() = AutoMode.default.command()

    val selectedAutonomousCommand: Command
        get() = autoModeChooser.selected?.command?.invoke() ?: defaultAutonomousCommand

    /** Example static factory for an autonomous command.  */

    init {
        val tab = Shuffleboard.getTab("Autonomous")
        tab.add("Auto Chooser", autoModeChooser)
    }

    private fun examplePath(): Command {
        // return SwerveSubsystem.getAutonomousCommand("examplePath", true)
        return PrintCommand("Example Path")
    }


    private fun midToTopNote(): Command {
        return SwerveSubsystem.getAutonomousCommand("Mid_To_TopNote", true)
    }

    private fun shootStill(): Command {
        return SequentialCommandGroup(
            ParallelCommandGroup(
                Commands.runOnce(
                    {LauncherSubsystem.state = LauncherSubsystem.State.PRIMED}
                ),
                PrimeLauncherCommand(),
                QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION)
            ),
            ReadyFireCommand()
        )
    }


    /**
     * An enumeration of the available autonomous modes. It provides an easy
     * way to manage all our autonomous modes. The [autoModeChooser] iterates
     * over its values, adding each value to the chooser.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    @Suppress("unused")
    private enum class AutoMode(val optionName: String, val command: () -> Command) {
        MID_TO_TOPNOTE("Mid to Top Note", { midToTopNote() }),
        EXAMPLE_PATH("Example Path", { examplePath() }),
        SHOOT_STILL("Shoot Still", { shootStill() })
        ;

        companion object {
            /** The default auto mode. */
            val default = EXAMPLE_PATH
        }
    }
}
