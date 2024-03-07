package frc.robot.commands.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem

class QuickPivotCommand(target: Double) : Command() {
    private val pivotSubsystem = PivotSubsystem
    private val target: Double
    private var direction: Boolean = false


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.target = target
    }

    override fun initialize() {
        direction = pivotSubsystem.getRelativePosition() > target
    }

    override fun execute() {
        if(pivotSubsystem.getRelativePosition() > target){
            pivotSubsystem.pivotMotor.set(-0.2)
        } else {
            pivotSubsystem.pivotMotor.set(0.2)
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if(direction){
            return pivotSubsystem.getRelativePosition() < target
        } else {
            return pivotSubsystem.getRelativePosition() > target
        }
    }

    override fun end(interrupted: Boolean) {
//        val hold = HoldTargetCommand(target)
//        hold.schedule()
    }
}
