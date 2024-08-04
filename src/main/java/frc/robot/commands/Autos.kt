package frc.robot.commands

import com.choreo.lib.Choreo
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.SwerveSubsystem

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
        val pathList = Choreo.getTrajectoryGroup("3P_SP_N123")


        return Commands.runOnce({
            pathList.forEachIndexed { index, path ->
                println("$index: $path.")
            }
        })

//        return Commands.sequence(
//            PrintCommand(pathList.size.toString()),
//            SwerveSubsystem.getPathCommand(pathList[0], true),
//            Commands.waitSeconds(2.0),
//            SwerveSubsystem.getPathCommand(pathList[1]),
//            Commands.waitSeconds(2.0),
//            SwerveSubsystem.getPathCommand(pathList[2])
//        )
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
        EXAMPLE_PATH("Example Path", { examplePath() }),
        ;

        companion object {
            /** The default auto mode. */
            val default = EXAMPLE_PATH
        }
    }
}
