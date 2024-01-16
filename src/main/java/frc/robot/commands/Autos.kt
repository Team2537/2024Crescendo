package frc.robot.commands

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem

object Autos {
    private val autoModeChooser =
        SendableChooser<AutoMode>().apply {
            AutoMode.values().forEach { addOption(it.optionName, it) }
            setDefaultOption(AutoMode.default.optionName, AutoMode.default)
        }

    val defaultAutonomousCommand: Command
        get() = AutoMode.default.command

    val selectedAutonomousCommand: Command
        get() = autoModeChooser.selected?.command ?: defaultAutonomousCommand

    /** Example static factory for an autonomous command.  */

    init {
        val tab = Shuffleboard.getTab("Autonomous")
        tab.add(autoModeChooser)
    }

    private fun examplePath(): Command {
        return SwerveSubsystem.getAutonomousCommand("examplePath", true)
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
    private enum class AutoMode(val optionName: String, val command: Command) {
        EXAMPLE_PATH("Example Path", examplePath()),
        ;

        companion object {
            /** The default auto mode. */
            val default = EXAMPLE_PATH
        }
    }
}
