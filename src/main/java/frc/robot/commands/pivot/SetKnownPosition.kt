package frc.robot.commands.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem
import lib.math.units.Rotation

/**
 * A command that sets the pivot to a known position.
 *
 * @param position the position to set the pivot to
 */
class SetKnownPosition(position: Rotation) : Command() {
    // This is somewhat of demonstration of the new simplified interface of the [PivotSubsystem] should
    // look like in use/how it should work: simply set position and update
    private val pivotSubsystem = PivotSubsystem
    private val position: Rotation

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.position = position
    }

    /**
     * Set the pivot encoder to the desired position
     */
    override fun initialize() {
        pivotSubsystem.position = position
    }

    override fun execute() {
        pivotSubsystem.tryUpdate()
    }

    override fun isFinished(): Boolean {
        return pivotSubsystem.isFinished
    }
}
