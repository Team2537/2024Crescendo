package frc.robot.commands.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem

/**
 * A command that sets the pivot to a known position.
 *
 * @param position the position to set the pivot to
 */
class SetKnownPosition(position: Double) : Command() {
    private val pivotSubsystem = PivotSubsystem
    private val position: Double

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.position = position
    }

    /**
     * Set the pivot encoder to the desired position
     */
    override fun initialize() {
        pivotSubsystem.relativeEncoder.setPosition(position)
    }

    /**
     * This command is finished instantly
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true
    }
}
