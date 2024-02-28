package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem

class PullNoteCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        launcherSubsystem.setRollerSpeed(-0.1)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
