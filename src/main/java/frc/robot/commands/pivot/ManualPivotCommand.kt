package frc.robot.commands.pivot

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem
import java.util.function.DoubleSupplier

class ManualPivotCommand(voltage: DoubleSupplier) : Command() {
    private val pivotSubsystem = PivotSubsystem
    private val voltage: DoubleSupplier


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.voltage = voltage
    }

    override fun initialize() {}

    override fun execute() {
        pivotSubsystem.setVoltage(voltage.asDouble)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
