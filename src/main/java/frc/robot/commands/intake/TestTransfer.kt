package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem

class TestTransfer : Command() {
    private val intakeSubsystem = IntakeSubsystem

    private val startTransferMotorSpeed = 0.2
    private val maxTransferMotorSpeed = 1.0
    private val transferMotorStep = 0.0016 // takes 10 seconds (500 * 20ms)

    private var currentTransferMotorSpeed = 0.0

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        // set motor speed to start speed
        currentTransferMotorSpeed = startTransferMotorSpeed
    }

    override fun execute() {
        // first, set motor speed to speed stored in current
        intakeSubsystem.transferMotor.set(currentTransferMotorSpeed)
        // next, use a ramp function to update current to a new value
        rampUpMotorSpeed()
    }

    private fun rampUpMotorSpeed() {
        // if ramp not over
        if(currentTransferMotorSpeed < maxTransferMotorSpeed) {
            // increment motor speed by step
            currentTransferMotorSpeed += transferMotorStep
        }
    }

    override fun isFinished(): Boolean {
        // this command should continue operating until it is stopped manually
        return false
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.transferMotor.stopMotor()
    }
}
