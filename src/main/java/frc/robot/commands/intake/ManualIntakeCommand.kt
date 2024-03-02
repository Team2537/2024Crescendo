package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem
import java.util.function.DoubleSupplier

class ManualIntakeCommand(
    intakeControl: DoubleSupplier,
    transferControl: DoubleSupplier
) : Command() {
    private val intakeSubsystem = IntakeSubsystem
    private val intakeControl: DoubleSupplier
    private val transferControl: DoubleSupplier

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
        this.intakeControl = intakeControl
        this.transferControl = transferControl
    }

    override fun initialize() {}

    override fun execute() {
        intakeSubsystem.intakeMotor.set(intakeControl.asDouble)
        intakeSubsystem.transferMotor.set(transferControl.asDouble)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
