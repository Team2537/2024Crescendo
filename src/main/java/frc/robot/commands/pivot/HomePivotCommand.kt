package frc.robot.commands.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem

/**
 * A command that homes the pivot subsystem by moving the pivot motor until the homing sensor is triggered.

 */
class HomePivotCommand : Command() {
    /** The subsystem that this command runs on. */
    private val pivotSubsystem = PivotSubsystem
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
    }

    /**
     * Set the pivot to move up at a constant speed
     */
    override fun initialize() {
        pivotSubsystem.setRawSpeed(-0.2)
    }

    /**
     * Finish the command when the homing sensor is triggered
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return pivotSubsystem.isUpright
    }

    /**
     * Stop the pivot motor when the command is interrupted or canceled
     * Zero the encoder
     */
    override fun end(interrupted: Boolean) {
        pivotSubsystem.stop()
        pivotSubsystem.zeroEncoder()
        println("Zeroing Encoder")
    }
}
