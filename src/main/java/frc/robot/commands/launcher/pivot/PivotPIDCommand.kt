package frc.robot.commands.launcher.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem

class PivotPIDCommand(target: Double) : Command() {
    private val pivotSubsystem = PivotSubsystem
    private val target: Double

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.target = target
    }

    override fun initialize() {
    }

    override fun execute() {
        pivotSubsystem.setTargetPosition(target)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
