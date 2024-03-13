package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Constants
import frc.robot.commands.launcher.PrimeLauncherCommand
import frc.robot.commands.launcher.ReadyFireCommand
import frc.robot.commands.pivot.HomePivotCommand
import frc.robot.commands.pivot.QuickPivotCommand

object Sequences {
    fun autoLaunch(): Command {
        return SequentialCommandGroup(
            HomePivotCommand(),
            ParallelCommandGroup(
                Commands.runOnce(
                    {LauncherSubsystem.state = LauncherSubsystem.State.PRIMED}
                ),
                PrimeLauncherCommand()
//                QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION, true)
            ),
            ReadyFireCommand()
        )
    }

    fun debugPrintSequence(): Command {
        return SequentialCommandGroup(
            PrintCommand("Test One"),
            WaitCommand(0.5),
            PrintCommand("Test Two"),
            WaitCommand(0.5),
            PrintCommand("Test Three"),
            WaitCommand(0.5)
        )
    }
}