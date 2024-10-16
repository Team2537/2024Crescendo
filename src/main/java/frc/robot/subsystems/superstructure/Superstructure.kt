package frc.robot.subsystems.superstructure

import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.superstructure.launcher.flywheels.Flywheels
import frc.robot.subsystems.superstructure.launcher.roller.Roller
import frc.robot.subsystems.superstructure.pivot.Pivot
import lib.math.units.degrees
import lib.math.units.rpm
import lib.not

class Superstructure {
    private val roller = Roller()
    private val flywheels = Flywheels()
    private val pivot = Pivot()

    fun getSubwooferShotCommand() =
        Commands.sequence(
            pivot.getSendToPositionCommand(subwooferShotSetpoint),
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
            pivot.getSendToPositionCommand(ampShotSetpoint),
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
            pivot.getSendToPositionCommand(intakePosition),
            Commands.parallel(
                flywheels.getStopCommand(),
                roller.getPullNoteCommand()
            )
        )

    fun getEjectCommand() =
        Commands.sequence(
            pivot.getSendToPositionCommand(intakePosition),
            roller.getEjectCommand()
        )

    fun getHomeCommand() = pivot.getHomeCommand()


    companion object {
        val subwooferShotSetpoint = 31.5.degrees
        val intakePosition = 16.5.degrees
        val ampShotSetpoint = 90.0.degrees
    }
}