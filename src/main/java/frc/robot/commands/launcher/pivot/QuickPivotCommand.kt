package frc.robot.commands.launcher.pivot

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem
import lib.math.units.Rotation
import lib.math.units.into
import lib.math.units.rotations

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
        direction = pivotSubsystem.getPosition() > target
    }

    override fun execute() {
        if(pivotSubsystem.getPosition() > target){
            pivotSubsystem.rawMotorSpeed(-0.15)
        } else {
            pivotSubsystem.rawMotorSpeed(0.15)
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if(direction){
            return pivotSubsystem.getPosition() < target
        } else {
            return pivotSubsystem.getPosition() > target
        }
    }

    override fun end(interrupted: Boolean) {
        val pidCommand = PivotPIDCommand(target)
        pidCommand.schedule()
    }
}
