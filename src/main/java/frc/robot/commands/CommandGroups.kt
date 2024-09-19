package frc.robot.commands
//
//import edu.wpi.first.wpilibj2.command.Command
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
//import frc.robot.commands.intake.ToggleIntakeCommand
//import frc.robot.commands.launcher.IntakeNoteCommand

@Deprecated("Oopsy doopsy, this file is deprecated. Please use the new Autos.kt file in the same directory.")
object CommandGroups {
//    fun autoLaunch(): Command {
//        return ParallelDeadlineGroup(
//            WaitCommand(4.0),
//            SequentialCommandGroup(
//                HomePivotCommand(),
//                ParallelCommandGroup(
//                    Commands.runOnce(
//                        {LauncherSubsystem.state = LauncherSubsystem.State.PRIMED}
//                    ),
//                    PrimeLauncherCommand(),
//                    QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION, true)
//                ),
//                ReadyFireCommand()
//            )
//        )
//    }

//    fun intakeGroup(): Command {
//        return ParallelCommandGroup(
//            ToggleIntakeCommand(),
//            IntakeNoteCommand()
//        )
//    }
}