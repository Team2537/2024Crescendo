package frc.robot.commands.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem

class HomePivotCommand : Command() {
    private val pivotSubsystem = PivotSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
    }

    override fun initialize() {
        pivotSubsystem.setRawSpeed(-0.2)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return pivotSubsystem.getHomingSensor()
    }

    override fun end(interrupted: Boolean) {
        pivotSubsystem.stop()
        pivotSubsystem.zeroEncoder()
        println("Zeroing Encoder")
    }
}
