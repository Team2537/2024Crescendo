package frc.robot.commands

import com.choreo.lib.Choreo
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.subsystems.SwerveSubsystem
import lib.auto.ChoreoAuto
import lib.auto.getPath
import java.util.*
import java.util.function.Supplier

object Autos {
    private val autoModeChooser =
        SendableChooser<AutoMode>().apply {
            AutoMode.entries.forEach { addOption(it.optionName, it) }
            setDefaultOption(AutoMode.default.optionName, AutoMode.default)
        }

    val defaultAutonomousCommand: Command
        get() = AutoMode.default.routine.get()

    val selectedAutonomousCommand: Command
        get() = autoModeChooser.selected.routine.get() ?: defaultAutonomousCommand


    /** Example static factory for an autonomous command.  */

    init {
        val tab = Shuffleboard.getTab("Autonomous")
        tab.add("Auto Chooser", autoModeChooser)
    }


    val ThreePieceSpeakerPath = Choreo.getTrajectoryGroup("3P_SP_N123")
    val ThreePieceSpeakerRed = SequentialCommandGroup(
        InstantCommand({RobotContainer.swerve.resetOdometry(ThreePieceSpeakerPath.first().flippedInitialPose)}),
        getPath(ThreePieceSpeakerPath[0], true, RobotContainer.swerve),
        WaitCommand(2.0),
        getPath(ThreePieceSpeakerPath[1], true, RobotContainer.swerve),
    )
    val ThreePieceSpeakerBlue = SequentialCommandGroup(
        InstantCommand({RobotContainer.swerve.resetOdometry(ThreePieceSpeakerPath.first().initialPose)}),
        getPath(ThreePieceSpeakerPath[0], false, RobotContainer.swerve),
        WaitCommand(2.0),
        getPath(ThreePieceSpeakerPath[1], false, RobotContainer.swerve),
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
    private enum class AutoMode(val optionName: String, val routine: Supplier<Command>) {
        EXAMPLE_PATH("Example Path", { ThreePieceSpeakerRed }),
        ;

        companion object {
            /** The default auto mode. */
            val default = EXAMPLE_PATH
        }
    }
}
