package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem
import edu.wpi.first.wpilibj.Timer

class IntakeCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem
    private val timer: Timer = Timer()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize() {
        timer.restart()
        launcherSubsystem.setRollerSpeed(-0.1)
    }


    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.5)
    }

    override fun end(interrupted: Boolean) {
        launcherSubsystem.intake()
    }
}
