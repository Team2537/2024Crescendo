package frc.robot.commands.climb

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem
import lib.near

class ClimbToTargetCommand(target: Double) : Command() {
    private val climbSubsystem = ClimbSubsystem
    private val target: Double
    // True is down, false is up
    private var direction: Boolean = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(climbSubsystem)
        this.target = target
    }

    override fun initialize() {
        if(climbSubsystem.leftMotor.encoder.position > target) {
            direction = true
        } else {
            direction = false
        }
    }

    override fun execute() {
        if(climbSubsystem.leftMotor.encoder.position > target){
            climbSubsystem.setRawSpeeds(-0.5)
        } else {
            climbSubsystem.setRawSpeeds(0.5)
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return climbSubsystem.leftMotor.encoder.position.near(target, 0.1)
    }

    override fun end(interrupted: Boolean) {}
}
