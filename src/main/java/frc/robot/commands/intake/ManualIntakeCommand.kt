package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem
import java.util.function.DoubleSupplier

/**
 * A command that allows manual control of the intake and transfer motors.
 *
 * @param intakeControl a supplier that returns the desired speed of the intake motor
 * @param transferControl a supplier that returns the desired speed of the transfer motor
 */
class ManualIntakeCommand(
    intakeControl: DoubleSupplier,
    transferControl: DoubleSupplier
) : Command() {
    /** The subsystem that this command runs on. */
    private val intakeSubsystem = IntakeSubsystem
    /** The supplier that returns the desired speed of the intake motor. */
    private val intakeControl: DoubleSupplier
    /** The supplier that returns the desired speed of the transfer motor. */
    private val transferControl: DoubleSupplier

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
        this.intakeControl = intakeControl
        this.transferControl = transferControl
    }

    /**
     * Set the speed of the intake and transfer motors to the desired speed
     * Runs every 20ms
     */
    override fun execute() {
        intakeSubsystem.intakeMotor.set(intakeControl.asDouble)
        intakeSubsystem.transferMotor.set(transferControl.asDouble)
    }

    /**
     * This command is never finished on its own.
     * It will be finished when the command is interrupted or canceled.
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }
}
