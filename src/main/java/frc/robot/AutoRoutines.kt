package frc.robot

import choreo.auto.AutoFactory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier

class AutoRoutines(val factory: AutoFactory) {
    private fun waitPrintCommand(time: Double, message: String) = Commands.parallel(
        WaitCommand(time),
        PrintCommand(message)
    )


    private val chooser = LoggedDashboardChooser<Supplier<Command>>("auto").apply {
        addDefaultOption("Four Note", Supplier { fourNote() })
        addOption("A3_CS", Supplier { A3_CS() })
    }

    val selectedRoutine: Command
            get() = chooser.get().get()

    fun fourNote(): Command {
        val CS_A1 = factory.trajectory("CS_A1", factory.voidLoop())
        val A1_CS = factory.trajectoryCommand("A1_CS")
        val CS_A2 = factory.trajectoryCommand("CS_A2")
        val A2_CS = factory.trajectoryCommand("A2_CS")
        val CS_A3 = factory.trajectoryCommand("CS_A3")
        val A3_CS = factory.trajectoryCommand("A3_CS")

        var startPose: Pose2d = Pose2d()
        CS_A1.initialPose.ifPresent { startPose = it }

        return Commands.sequence(
            InstantCommand({ Robot.drivebase.resetOdometry(startPose) }),
            waitPrintCommand(2.0, "Launching first note"),
            Commands.parallel(
                CS_A1.cmd(),
                PrintCommand("Intaking second note")
            ),
            A1_CS,
            waitPrintCommand(2.0, "Launching second note"),
            Commands.parallel(
                CS_A2,
                PrintCommand("Intaking third note")
            ),
            A2_CS,
            waitPrintCommand(2.0, "Launching third note"),
            Commands.parallel(
                CS_A3,
                PrintCommand("Intaking fourth note")
            ),
            A3_CS,
            waitPrintCommand(2.0, "Launching fourth note")
        )
    }

    fun A3_CS(): Command {
        val A3_CS = factory.trajectory("A3_CS", factory.voidLoop())

        var startPose: Pose2d = Pose2d()
        A3_CS.initialPose.ifPresent { startPose = it }

        return Commands.sequence(
            InstantCommand({ Robot.drivebase.resetOdometry(startPose) }),
            A3_CS.cmd()
        )
    }
}

