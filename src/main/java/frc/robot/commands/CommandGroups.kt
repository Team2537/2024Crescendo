package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.commands.intake.ToggleIntakeCommand

object CommandGroups {
//    fun autoLaunch(): Command {
//        return ParallelDeadlineGroup(
//            WaitCommand(4.0),
//            SequentialCommandGroup(
//                HomePivotCommand(),
//                ParallelCommandGroup(
//                    Commands.runOnce(
//                        {frc.robot.subsystems.LauncherSubsystem.state = frc.robot.subsystems.LauncherSubsystem.State.PRIMED}
//                    ),
//                    PrimeLauncherCommand(),
//                    QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION, true)
//                ),
//                ReadyFireCommand()
//            )
//        )
//    }

    fun intakeGroup(): Command {
        return ParallelCommandGroup(
            ToggleIntakeCommand(),
            IntakeNoteCommand()
        )
    }
}