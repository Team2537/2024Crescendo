package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem

/**
 * Command to slowly ramp up the speed of the transfer motor
 * Used to test the transfer motor and ensure it is functioning correctly
 */
class TestTransfer : Command() {
    /** The subsystem that this command runs on. */
    private val intakeSubsystem = IntakeSubsystem
    /** The speed to start the transfer motor at */
    private val startTransferMotorSpeed = 0.2
    /** The maximum speed the transfer motor should reach */
    private val maxTransferMotorSpeed = 1.0
    /** The step size to increment the transfer motor speed by */
    private val transferMotorStep = 0.0016 // takes 10 seconds (500 * 20ms)
    /** The current speed of the transfer motor */
    private var currentTransferMotorSpeed = 0.0

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    /**
     * Set the speed of the transfer motor to the start speed
     */
    override fun initialize() {
        // set motor speed to start speed
        currentTransferMotorSpeed = startTransferMotorSpeed
    }

    /**
     * Set the speed of the transfer motor to the current speed
     * Then increment the speed by the step size
     * Runs every 20ms
     */
    override fun execute() {
        // first, set motor speed to speed stored in current
        intakeSubsystem.transferMotor.set(currentTransferMotorSpeed)
        // next, use a ramp function to update current to a new value
        rampUpMotorSpeed()
    }

    /**
     * Increment the speed of the transfer motor by the step size
     */
    private fun rampUpMotorSpeed() {
        // if ramp not over
        if(currentTransferMotorSpeed < maxTransferMotorSpeed) {
            // increment motor speed by step
            currentTransferMotorSpeed += transferMotorStep
        }
    }

    /**
     * This command is never finished on its own.
     * It will be finished when the command is interrupted or canceled.
     */
    override fun isFinished(): Boolean {
        // this command should continue operating until it is stopped manually
        return false
    }

    /**
     * Stop the transfer motor when the command is interrupted or canceled
     */
    override fun end(interrupted: Boolean) {
        intakeSubsystem.transferMotor.stopMotor()
    }
}
