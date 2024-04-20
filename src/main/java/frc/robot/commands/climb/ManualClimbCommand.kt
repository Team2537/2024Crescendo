package frc.robot.commands.climb

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem
import java.util.function.DoubleSupplier

/**
 * A command that allows manual control of the climber arms
 * @param speed Double supplier that returns the speed to set the motors to
 */
class ManualClimbCommand(speed: DoubleSupplier) : Command() {
    private val climbSubsystem = ClimbSubsystem
    private val speed: DoubleSupplier


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(climbSubsystem)
        this.speed = speed
    }

    /**
     * Set the speed of the motors to the speed given by the speed supplier
     */
    override fun execute() {
        climbSubsystem.setRawSpeeds(speed.asDouble)
    }

    /**
     * This command is never finished on its own.
     * It will be finished when the command is interrupted or canceled.
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }
}
