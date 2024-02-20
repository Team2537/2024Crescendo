package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem

class IntakeCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize() {
        launcherSubsystem.intake()
    }

    override fun isFinished(): Boolean {
        return false
    }
}
