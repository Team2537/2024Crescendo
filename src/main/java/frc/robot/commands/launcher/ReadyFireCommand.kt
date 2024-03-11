package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem

class ReadyFireCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize()
    {
        launcherSubsystem.fire = true
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true
    }

    override fun end(interrupted: Boolean) {
        println("Firing the Launcher!")
    }
}
