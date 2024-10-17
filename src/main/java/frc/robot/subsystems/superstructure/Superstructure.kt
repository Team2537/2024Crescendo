package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitSeconds
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.subsystems.superstructure.launcher.flywheels.Flywheels
import frc.robot.subsystems.superstructure.launcher.roller.Roller
import frc.robot.subsystems.superstructure.pivot.Pivot
import lib.debug
import lib.math.units.degrees
import lib.math.units.rpm
import lib.not
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

    fun getSubwooferShotCommand() =
        Commands.sequence(
            pivot.getQuickAngleCommand(subwooferShotSetpoint),
            flywheels.getPrimeCommand(6500.0.rpm),
            flywheels.getWaitUntilReadyCommand(),
            roller.getPushNoteCommand(),
            Commands.waitUntil(!roller.isHoldingNote),
            Commands.waitSeconds(0.5),
            Commands.parallel(
                flywheels.getStopCommand(),
                roller.getStopCommand()
            )
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

    fun getIntakeCommand() =
        Commands.sequence(
            pivot.getQuickAngleCommand(intakePosition).debug("pivot"),
            Commands.parallel(
                flywheels.getStopCommand(),
                roller.getPullNoteCommand()
            )
        )

    fun resetPivotPosition() = pivot.getResetPositionCommand()

    fun getEjectCommand() =
        Commands.sequence(
            pivot.getQuickAngleCommand(intakePosition),
            roller.getEjectCommand()
        )

    fun getHomeCommand() = pivot.getSensorlessHomeCommand()

    fun getManualControlCommand(voltage: DoubleSupplier) = pivot.manualControl(voltage)

    companion object {
        val subwooferShotSetpoint = 31.5.degrees
        val intakePosition = 16.5.degrees
        val ampShotSetpoint = 90.0.degrees
    }
}