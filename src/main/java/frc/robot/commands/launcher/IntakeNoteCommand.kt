package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem
import edu.wpi.first.wpilibj.Timer

class IntakeNoteCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem
    private val timer: Timer = Timer()


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize() {
        timer.restart()
        launcherSubsystem.setRollerSpeed(0.1)
        launcherSubsystem.stopFlywheels()
    }

    override fun execute() {
        if(launcherSubsystem.getNoteTrigger().asBoolean
            && timer.hasElapsed(0.25) // TODO: Tune this value
            ) {
            launcherSubsystem.setRollerPosition(
                launcherSubsystem.getRollerPosition()
            )
        } else {
            timer.reset()
            launcherSubsystem.setRollerSpeed(0.1)
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(1.0) && launcherSubsystem.getNoteTrigger().asBoolean
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        launcherSubsystem.setRollerPosition(
            launcherSubsystem.getRollerPosition()
        )
    }
}
