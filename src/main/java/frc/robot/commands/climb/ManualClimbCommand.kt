package frc.robot.commands.climb

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem
import java.util.function.DoubleSupplier

class ManualClimbCommand(speed: DoubleSupplier) : Command() {
    private val climbSubsystem = ClimbSubsystem
    private val speed: DoubleSupplier


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(climbSubsystem)
        this.speed = speed
    }

    override fun initialize() {}

    override fun execute() {
        climbSubsystem.setArmSpeeds(speed.asDouble)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        climbSubsystem.setArmSpeeds(0.0)
    }
}
