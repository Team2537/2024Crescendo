package frc.robot.commands.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem

class HoldPositionCommand : Command() {
    private val pivotSubsystem = PivotSubsystem
    private var target: Double = 0.0


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
    }

    override fun initialize() {
        target = pivotSubsystem.getRelativePosition()
        println("Holding position at $target")
    }

    override fun execute() {
        pivotSubsystem.holdArm(target)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        pivotSubsystem.stop()
    }
}
