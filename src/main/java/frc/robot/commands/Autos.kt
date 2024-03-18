package frc.robot.commands

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants
import frc.robot.commands.intake.ToggleIntakeCommand
import frc.robot.commands.launcher.IntakeNoteCommand
import frc.robot.commands.launcher.LaunchCommand
import frc.robot.commands.pivot.HomePivotCommand
import frc.robot.commands.pivot.QuickPivotCommand
import frc.robot.subsystems.PivotSubsystem
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

        registerNamedCommands()
    }

    fun registerNamedCommands() {
//        NamedCommands.registerCommand("Auto Launch", Sequences.autoLaunch())
        NamedCommands.registerCommand("Intake", ToggleIntakeCommand())
        NamedCommands.registerCommand("Auto Aim", QuickPivotCommand(10.0, true, true))
        NamedCommands.registerCommand("Aim Subwoofer", QuickPivotCommand(
            Constants.PivotConstants.SUBWOOFER_POSITION, false, false
        ))
        NamedCommands.registerCommand("Shoot", LaunchCommand(
            { 1.0 },
            { true },
            { PivotSubsystem.getRelativePosition() },
            { false }
        ))
        NamedCommands.registerCommand("Home", HomePivotCommand())
        NamedCommands.registerCommand("Aim Intake", QuickPivotCommand(
            Constants.PivotConstants.INTAKE_POSITION, false, false
        ))
        NamedCommands.registerCommand("Pull Note", IntakeNoteCommand())
        NamedCommands.registerCommand("Intake Note", ToggleIntakeCommand())
    }

    private fun examplePath(): Command {
        // return SwerveSubsystem.getAutonomousCommand("examplePath", true)
        return PrintCommand("Example Path")
    }


//    private fun shootStill(): Command {
//        return SequentialCommandGroup(
//            HomePivotCommand(),
//            ParallelCommandGroup(
//                Commands.runOnce(
//                    {LauncherSubsystem.state = LauncherSubsystem.State.PRIMED}
//                ),
//                PrimeLauncherCommand(),
//                QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION, true)
//            ),
//            ReadyFireCommand()
//        )
//    }

    private fun testAuto(): Command {
        return SwerveSubsystem.getAutonomousCommand("Basic_Drive", true)
    }


    private fun shootDriveSource():Command {
        return SwerveSubsystem.getAutonomousCommand("Shoot_Drive_Source", true)
    }

    private fun shootAndDriveRight(): Command {
        return SwerveSubsystem.getAutonomousCommand("Shoot_And_Drive_Right", true)
    }

    private fun midToTopNote(): Command {
        return SwerveSubsystem.getAutonomousCommand("Mid_To_TopNote", true)
    }

    private fun onlyShoot(): Command {
        return SequentialCommandGroup(
            HomePivotCommand(),
            QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION, false, false),
            LaunchCommand(
                { 1.0 },
                { true },
                { PivotSubsystem.getRelativePosition() },
                { false }
            ),
        )
    }

    private fun twoNote(): Command {
        return SwerveSubsystem.getAutonomousCommand("Two_Note", true)
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
        TEST_AUTO("Test Auto", { testAuto() }),
        SHOOT_DRIVE_MID("Shoot & Drive Source", { shootDriveSource() }),
        SHOOT_DRIVE_RIGHT("Shoot & Drive Right", { shootAndDriveRight() }),
        SHOOT_DRIVE_LEFT("Shoot & Drive Left", { shootDriveSource() }),
        BASIC_SHOOT("Basic Shoot", { onlyShoot() }),
        TWO_NOTE("Two Note", {twoNote()})
        ;

        companion object {
            /** The default auto mode. */
            val default = EXAMPLE_PATH
        }
    }
}
