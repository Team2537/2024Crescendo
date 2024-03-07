package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem

class DriftTestCommand(directionTime: Double, speed: Double) : Command() {
    private val swerveSubsystem = SwerveSubsystem
    private val timer: Timer = Timer()
    private val directionTime: Double
    private val speed: Double

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
        this.directionTime = directionTime
        this.speed = speed
    }


    override fun initialize() {
        timer.restart()
    }

    override fun execute() {
        if(!timer.hasElapsed(directionTime)){
            swerveSubsystem.drive(Translation2d(speed, 0.0), 0.0, false)
        } else if (timer.hasElapsed(directionTime) && !timer.hasElapsed(directionTime*2)){
            swerveSubsystem.drive(Translation2d(-speed, 0.0), 0.0, false)
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return timer.hasElapsed(directionTime*2)
    }

    override fun end(interrupted: Boolean) {}
}
