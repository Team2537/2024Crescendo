package frc.robot.commands.intake

import LauncherSubsystem
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem

class FeedLauncherCommand : Command() {
    /** The subsystem that this command runs on. */
    private val intakeSubsystem = IntakeSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    /**
     * Set the speed of the transfer to a constant speed to feed notes into the launcher
     */
    override fun initialize() {
        intakeSubsystem.transferMotor.set(1.0)
    }


    /**
     * This command is never finished on its own.
     * It will be finished when the command is interrupted or canceled.
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    /**
     * Stop the transfer motor when the command is interrupted or canceled
     */
    override fun end(interrupted: Boolean) {
        intakeSubsystem.transferMotor.stopMotor()
        println("FeedLauncherCommand ended, Interrupted: $interrupted")
    }
}
