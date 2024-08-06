package frc.robot.commands

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import frc.robot.RobotContainer
import lib.auto.ChoreoAuto
import java.util.*
import java.util.function.Supplier

object Autos {
    private val autoModeChooser =
        SendableChooser<AutoMode>().apply {
            AutoMode.entries.forEach { addOption(it.optionName, it) }
            setDefaultOption(AutoMode.default.optionName, AutoMode.default)
        }

    private fun waitPrint(msg: String, wait: Double): Command {
        return Commands.parallel(
            Commands.print(msg),
            Commands.waitSeconds(wait)
        )
    }

    val defaultAutonomousCommand: ChoreoAuto
        get() = AutoMode.default.routine.get()

    val selectedAutonomousCommand: ChoreoAuto
        get() = autoModeChooser.selected.routine.get() ?: defaultAutonomousCommand


    /** Example static factory for an autonomous command.  */

    init {
        val tab = Shuffleboard.getTab("Autonomous")
        tab.add("Auto Chooser", autoModeChooser)
    }


    val ThreePieceSpeaker = ChoreoAuto(
        "3P_SP_N123",
        RobotContainer.swerve,
        true,
        mapOf(
            0 to Supplier {
                waitPrint("Shooting Note 1", 3.0)
            },
            1 to Supplier {
                waitPrint("Shooting Note 2", 3.0)
            },
            2 to Supplier {
                waitPrint("Shooting Note 3", 3.0)
            }
        ),
        mapOf(
            0 to Supplier { PrintCommand("Picking Up Note 1") },
            1 to Supplier { PrintCommand("Picking Up Note 2") },
            2 to Supplier { PrintCommand("Picking Up Note 3") }
        ),
        startCommand = Supplier { waitPrint("Shooting Stored Note", 2.0) }
    )


    /**
     * An enumeration of the available autonomous modes. It provides an easy
     * way to manage all our autonomous modes. The [autoModeChooser] iterates
     * over its values, adding each value to the chooser.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    @Suppress("unused")
    private enum class AutoMode(val optionName: String, val routine: Supplier<ChoreoAuto>) {
        EXAMPLE_PATH("Example Path", { ThreePieceSpeaker }),
        ;

        companion object {
            /** The default auto mode. */
            val default = EXAMPLE_PATH
        }
    }
}
