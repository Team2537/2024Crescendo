package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LauncherSubsystem

class TestLaunchCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize() {
        launcherSubsystem.setLaunchVelocity(6700.0)
    }

    override fun execute() {

    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
