package frc.robot.commands.pivot

import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem
import java.util.function.DoubleSupplier

/**
 * A command that allows manual control of the pivot motor.
 *
 * @param voltage a supplier that returns the desired voltage of the pivot motor
 */
class ManualPivotCommand(voltage: DoubleSupplier) : Command() {
    /** The subsystem that this command runs on. */
    private val pivotSubsystem = PivotSubsystem
    /** The supplier that returns the desired voltage of the pivot motor. */
    private val voltage: DoubleSupplier


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.voltage = voltage
    }

    /**
     * Set the voltage of the pivot motor to the desired voltage
     * Runs every 20ms
     */
    override fun execute() {
        pivotSubsystem.setVoltage(Volts.of(voltage.asDouble))
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
