package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem

/**
 * Command to toggle the intake and transfer motors on and off
 */
class ToggleIntakeCommand : Command() {
    /** The subsystem that this command runs on. */
    private val intakeSubsystem = IntakeSubsystem

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    /**
     * Set the speed of the intake and transfer motors to pull notes into the robot
     */
    override fun initialize() {
        intakeSubsystem.intakeMotor.set(-1.0)
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
     * Stop the intake and transfer motors when the command is interrupted or canceled
     */
    override fun end(interrupted: Boolean) {
        intakeSubsystem.intakeMotor.stopMotor()
        intakeSubsystem.transferMotor.stopMotor()
        if(interrupted){
            println("Interrupted")
        }
    }
}
