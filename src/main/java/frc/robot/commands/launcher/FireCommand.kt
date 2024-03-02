package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem

class FireCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize() {
        launcherSubsystem.setRollerSpeed(-1.0)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        launcherSubsystem.stop()
        launcherSubsystem.stopRollers()
    }

}
