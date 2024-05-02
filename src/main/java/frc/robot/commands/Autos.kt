package frc.robot.commands

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.LimelightSubsystem
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

    private fun registerNamedCommands() {}

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


    private fun shootDriveSource(): Command {
        return SwerveSubsystem.getAutonomousCommand("Shoot_Drive_Source", true)
    }

    private fun shootAndDriveRight(): Command {
        return SwerveSubsystem.getAutonomousCommand("Shoot_And_Drive_Right", true)
    }

    private fun shootAndSteal(): Command {
        return SwerveSubsystem.getAutonomousCommand("Steal_MidAmp", true)
    }

    private fun midToTopNote(): Command {
        return SwerveSubsystem.getAutonomousCommand("Mid_To_TopNote", true)
    }

    private fun twoNote(): Command {
        return SwerveSubsystem.getAutonomousCommand("Two_Note", true)
    }

    private fun threeNoteCenter(): Command {
        return SwerveSubsystem.getAutonomousCommand("Three_Note", true)
    }

    private fun threeNoteAmp(): Command {
        return SwerveSubsystem.getAutonomousCommand("Three_Note_Amp", true)
    }

    private fun threeNoteSource(): Command {
        return SwerveSubsystem.getAutonomousCommand("Source_ThreeNote", true)
    }

//    private fun intakeTest(): Command {
//        return SwerveSubsystem.getAutonomousCommand("Two_Note_Intake", true)
//    }
//
//    private fun twoNoteAmp(): Command {
//        return SwerveSubsystem.getAutonomousCommand("Two_Note_Amp", true)
//    }


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
        SHOOT_DRIVE_RIGHT("Shoot & Drive Amp", { shootAndDriveRight() }),
        TWO_NOTE("Two Note", { twoNote() }),
        THREE_NOTE_CENTER("Three Note Center", { threeNoteCenter() }),
        THREE_NOTE_AMP("Three Note Amp", { threeNoteAmp() }),
        THREE_NOTE_SOURCE("Three Note Source", { threeNoteSource() }),
        SHOOT_STEAL("Shoot & Steal", { shootAndSteal() })
        ;

        companion object {
            /** The default auto mode. */
            val default = EXAMPLE_PATH
        }
    }
}
