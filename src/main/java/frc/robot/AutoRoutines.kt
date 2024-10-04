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
import kotlin.jvm.optionals.getOrElse

class AutoRoutines(val factory: AutoFactory) {
    private fun waitPrintCommand(time: Double, message: String) = Commands.parallel(
        WaitCommand(time),
        PrintCommand(message)
    )


    private val chooser = LoggedDashboardChooser<Supplier<Command>>("auto").apply {
        addDefaultOption("Four Note (Dumb)", Supplier { dumbFourNoteA1_A3() })
        addOption("Four Note (Smart)", Supplier { smartMaxNotesA1_A3() })
    }

    val selectedRoutine: Command
        get() = chooser.get().get()

    fun dumbFourNoteA1_A3(): Command {
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

    fun smartMaxNotesA1_A3(): Command {
        val loop = factory.newLoop("smartMaxNotesA1_A3")

        val CS_A1 = factory.trajectory("CS_A1", loop)
        val CS_A2 = factory.trajectory("CS_A2", loop)
        val CS_A3 = factory.trajectory("CS_A3", loop)

        val A1_CS = factory.trajectory("A1_CS", loop)
        val A2_CS = factory.trajectory("A2_CS", loop)
        val A3_CS = factory.trajectory("A3_CS", loop)

        val A1_A2 = factory.trajectory("A1_A2", loop)
        val A2_A3 = factory.trajectory("A2_A3", loop)

        loop.enabled()
            .onTrue(
                InstantCommand({ Robot.drivebase.resetOdometry(CS_A1.initialPose.getOrElse { loop.kill(); return@getOrElse Pose2d() }) })
                    .andThen(waitPrintCommand(2.0, "Launching first note"))
                    .andThen(
                        Commands.parallel(
                            waitPrintCommand(2.0, "Intaking second note"),
                            CS_A1.cmd()
                        )
                    ).withName("fourNoteA1_A3 Entry")
            )

        // Either try to get the next note if you didn't pick one up, or go shoot the stored one
        CS_A1.done()
            .and( { false } ) // TODO: Add a condition for the next note
            .onTrue(
                A1_CS.cmd()
                    .andThen(waitPrintCommand(2.0, "Launching second note"))
                    .andThen(
                        Commands.parallel(
                            waitPrintCommand(2.0, "Intaking third note"),
                            CS_A2.cmd()
                        )
                    )
            )
            .onFalse(
                A1_A2.cmd()
            )

        CS_A2.done()
            .and( { false } ) // TODO: Add a condition for the next note
            .onTrue(
                A2_CS.cmd()
                    .andThen(waitPrintCommand(2.0, "Launching third note"))
                    .andThen(
                        Commands.parallel(
                            waitPrintCommand(2.0, "Intaking fourth note"),
                            CS_A3.cmd()
                        )
                    )
            )
            .onFalse(
                A2_A3.cmd()
            )

        CS_A3.done()
            .and( { false } ) // TODO: Add a condition for the next note
            .onTrue(
                A3_CS.cmd()
                    .andThen(waitPrintCommand(2.0, "Launching fourth note"))
            )

        A1_A2.done()
            .and({ false }) // TODO: Add a condition for the next note
            .onTrue(
                A2_CS.cmd()
                    .andThen(waitPrintCommand(2.0, "Launching second note"))
                    .andThen(
                        Commands.parallel(
                            waitPrintCommand(2.0, "Intaking third note"),
                            CS_A3.cmd()
                        )
                    )
            )
            .onFalse(
                A1_CS.cmd()
            )

        A2_A3.done()
            .and({ false }) // TODO: Add a condition for the next note
            .onTrue(
                A3_CS.cmd()
                    .andThen(waitPrintCommand(2.0, "Launching third note"))
            )

        return loop.cmd()
    }
}

