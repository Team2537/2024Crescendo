package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem

class PrimeLauncherCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize() {
        launcherSubsystem.setLauncherSpeed(1.0)
    }

    override fun isFinished(): Boolean {
        return launcherSubsystem.state == LauncherSubsystem.State.AT_SPEED
    }

    override fun end(interrupted: Boolean)
    {
        println("Ending")
    }
}
