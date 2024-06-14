package frc.robot.commands.launcher

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LauncherSubsystem
import lib.math.units.inInches

class LoadNoteCommand(
    private val subsystem: LauncherSubsystem
) : Command() {
    private val timer: Timer = Timer()

    init { addRequirements(subsystem) }

    override fun initialize() {
        subsystem.stopFlywheels()
        subsystem.setRollerBrake(false)

        timer.restart()
        subsystem.setRawRollerVoltage(-1.2)
    }

    override fun execute() {
        if(!subsystem.noteDetected) {
            timer.reset()
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.2) && subsystem.noteDetected
    }

    override fun end(interrupted: Boolean) {
        subsystem.stopRoller()
        timer.stop()
        subsystem.setRollerBrake(true)
        subsystem.setRollerPosition(subsystem.getRollerPosition().plus(2.inInches))
    }
}