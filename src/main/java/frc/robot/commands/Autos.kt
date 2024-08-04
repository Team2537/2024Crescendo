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
            0 to Supplier { PrintCommand("Picking up note 1") },
            1 to Supplier { PrintCommand("Picking Up Note 2") },
        )
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
