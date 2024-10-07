package frc.robot

import choreo.auto.AutoFactory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.either
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.swerve.Drivebase
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrElse

class AutoRoutines(
    val factory: AutoFactory,
    val drivebase: Drivebase,
) {
    private fun waitPrintCommand(time: Double, message: String) = parallel(
        WaitCommand(time),
        PrintCommand(message)
    )


    private val chooser = LoggedDashboardChooser<Supplier<Command>>("auto").apply {
        addOption("Four Note (Dumb)", Supplier { dumbFourNoteA1_A3() })
        addDefaultOption("Four Note (Smart)", Supplier { smartMaxNotesA1_A3() })
    }

    val selectedRoutine: Command
        get() = chooser.get().get()

    init {
        if (RobotBase.isSimulation()) {
            SmartDashboard.putBoolean("notes/A1", false)
            SmartDashboard.putBoolean("notes/A2", false)
            SmartDashboard.putBoolean("notes/A3", false)
        }
    }

    fun dumbFourNoteA1_A3(): Command {
        val CS_A1 = factory.trajectory("CS_A1", factory.voidLoop())
        val A1_CS = factory.trajectoryCommand("A1_CS")
        val CS_A2 = factory.trajectoryCommand("CS_A2")
        val A2_CS = factory.trajectoryCommand("A2_CS")
        val CS_A3 = factory.trajectoryCommand("CS_A3")
        val A3_CS = factory.trajectoryCommand("A3_CS")

        var startPose: Pose2d = Pose2d()
        CS_A1.initialPose.ifPresent { startPose = it }

        return sequence(
            InstantCommand({ drivebase.resetOdometry(startPose) }),
            waitPrintCommand(2.0, "Launching first note"),
            parallel(
                CS_A1.cmd(),
                PrintCommand("Intaking second note")
            ),
            A1_CS,
            waitPrintCommand(2.0, "Launching second note"),
            parallel(
                CS_A2,
                PrintCommand("Intaking third note")
            ),
            A2_CS,
            waitPrintCommand(2.0, "Launching third note"),
            parallel(
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
                InstantCommand({ drivebase.resetOdometry(CS_A1.initialPose.getOrElse { loop.kill(); return@getOrElse Pose2d() }) })
                    .andThen(waitPrintCommand(2.0, "Launching first note"))
                    .andThen(
                        parallel(
                            waitPrintCommand(2.0, "Intaking second note"),
                            CS_A1.cmd()
                        )
                    ).withName("fourNoteA1_A3 Entry")
            )

        // Either try to get the next note if you didn't pick one up, or go shoot the stored one
        CS_A1.done()
            .onTrue(
                either(
                    A1_CS.cmd()
                        .andThen(waitPrintCommand(2.0, "Launching second note"))
                        .andThen(
                            Commands.parallel(
                                waitPrintCommand(2.0, "Intaking third note"),
                                CS_A2.cmd()
                            )
                        ),
                    A1_A2.cmd()
                ) { if (RobotBase.isSimulation()) SmartDashboard.getBoolean("notes/A1", false) else false }
            )


        CS_A2.done()
            .onTrue(
                either(
                    A2_CS.cmd()
                        .andThen(waitPrintCommand(2.0, "Launching third note"))
                        .andThen(
                            Commands.parallel(
                                waitPrintCommand(2.0, "Intaking fourth note"),
                                CS_A3.cmd()
                            )
                        ),
                    A2_A3.cmd()
                ) { if (RobotBase.isSimulation()) SmartDashboard.getBoolean("notes/A2", false) else false }
            )

        CS_A3.done().and { if (RobotBase.isSimulation()) SmartDashboard.getBoolean("notes/A3", false) else false }
            .onTrue(
                A3_CS.cmd()
                    .andThen(waitPrintCommand(2.0, "Launching fourth note"))
            )

        A1_A2.done()
            .onTrue(
                either(
                    A2_CS.cmd()
                        .andThen(waitPrintCommand(2.0, "Launching second note"))
                        .andThen(
                            parallel(
                                waitPrintCommand(2.0, "Intaking third note"),
                                CS_A3.cmd()
                            )
                        ),
                    A2_A3.cmd()
                ) { if (RobotBase.isSimulation()) SmartDashboard.getBoolean("notes/A2", false) else false }
            )



        A2_A3.done().and { if (RobotBase.isSimulation()) SmartDashboard.getBoolean("notes/A3", false) else false }
            .onTrue(
                A3_CS.cmd()
                    .andThen(waitPrintCommand(2.0, "Launching third note"))
            )

        return loop.cmd()
    }
}

