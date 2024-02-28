package frc.robot.commands.launcher.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem
import java.util.function.DoubleSupplier

class ManualPivotCommand(speed: DoubleSupplier) : Command() {
    private val pivotSubsystem = PivotSubsystem
    private val speed: DoubleSupplier


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.speed = speed
    }

    override fun initialize() {
        println("we movin")
    }

    override fun execute() {
        pivotSubsystem.rawMotorSpeed(speed.asDouble)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        println("we stoppin")
    }
}
