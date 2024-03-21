package frc.robot.commands.swerve

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem

class TestRawVoltage : Command() {
    private val swerveSubsystem = SwerveSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        swerveSubsystem.setRawMotorVoltage(3.0)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        swerveSubsystem.setRawMotorVoltage(0.0)
    }
}
