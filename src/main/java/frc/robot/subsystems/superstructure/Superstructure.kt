package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitSeconds
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.subsystems.superstructure.launcher.flywheels.Flywheels
import frc.robot.subsystems.superstructure.launcher.roller.Roller
import frc.robot.subsystems.superstructure.pivot.Pivot
import lib.math.units.degrees
import lib.math.units.rpm
import lib.not
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class Superstructure {
    val roller = Roller()
    val flywheels = Flywheels()
    val pivot = Pivot()

    fun getPivotIntake() =
        pivot.getQuickAngleCommand(intakePosition)

    fun getTestShotCommand() =
        Commands.sequence(
            flywheels.getPrimeCommand(6000.0.rpm),
            flywheels.getWaitUntilReadyCommand(),
            PrintCommand("flywheel primed"),
            roller.getPushNoteCommand(),
            Commands.waitUntil(!roller.isHoldingNote),
            Commands.waitSeconds(0.5),
            Commands.parallel(
                flywheels.getStopCommand(),
                roller.getStopCommand()
            )
        ).onlyIf(roller.isHoldingNote)

    fun getSubwooferShotCommand(trigger: BooleanSupplier) =
        Commands.sequence(
            pivot.getSensorlessHomeCommand().debug(),
            Commands.print("Homing Finished"),
            flywheels.getPrimeCommand(6000.0.rpm).debug(),
            flywheels.getWaitUntilReadyCommand().debug(),
            waitUntil(trigger).debug(),
            roller.getPushNoteCommand().debug(),
            Commands.waitUntil(!roller.isHoldingNote).debug(),
            Commands.waitSeconds(0.5).debug(),
            Commands.parallel(
                flywheels.getStopCommand(),
                roller.getStopCommand()
            ).debug()
        ).onlyIf(roller.isHoldingNote)

    fun getAmpShotCommand() =
        Commands.sequence(
            pivot.getQuickAngleCommand(ampShotSetpoint),
            flywheels.getPrimeCommand(1500.0.rpm),
            flywheels.getWaitUntilReadyCommand(),
            roller.getPushNoteCommand(Volts.of(6.0)),
            Commands.waitUntil(roller.isHoldingNote),
            Commands.waitSeconds(0.5),
            Commands.parallel(
                flywheels.getStopCommand(),
                roller.getStopCommand()
            )
        ).onlyIf(roller.isHoldingNote)

    fun getIntakePivotCommand(endTrigger: BooleanSupplier) =
        pivot.getSendToPositionCommand(intakePosition)

    fun getPullNoteCommand(distance: Measure<Distance> = Inches.of(7.0)) =
        roller.getPullNoteCommand(distance)

    fun resetPivotPosition() = pivot.getResetPositionCommand()

    fun getConstantPullNote() =
        Commands.sequence(
            roller.runOnce { roller.rollerIO.setVoltage(Units.Volts.of(1.0)) },
            waitUntil(roller.isHoldingNote),
            waitSeconds(0.25),
            roller.getStopCommand()
        )

    fun getRetractNote() = getPullNoteCommand(Units.Inches.of(-1.0))

    fun getEjectCommand() =
        Commands.sequence(
            pivot.getQuickAngleCommand(intakePosition),
            roller.getEjectCommand()
        )

    fun getHomeCommand() = pivot.getSensorlessHomeCommand()

    fun getManualControlCommand(voltage: DoubleSupplier) = pivot.manualControl(voltage)

    companion object {
        val subwooferShotSetpoint = 5.degrees
        val intakePosition = 16.5.degrees
        val ampShotSetpoint = 90.0.degrees
    }

    fun Command.debug(msg: String = "${this.name} has ended"): Command {
        return this.andThen(PrintCommand(msg))
    }
}