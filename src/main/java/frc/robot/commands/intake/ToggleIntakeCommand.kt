package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem

class ToggleIntakeCommand : Command() {
    private val intakeSubsystem = IntakeSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        intakeSubsystem.intakeMotor.set(-1.0)
        intakeSubsystem.transferMotor.set(1.0)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.intakeMotor.stopMotor()
        intakeSubsystem.transferMotor.stopMotor()
        if(interrupted){
            println("Interrupted")
        }
    }
}
