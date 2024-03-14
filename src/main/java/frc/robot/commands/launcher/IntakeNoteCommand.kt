package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

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
       launcherSubsystem.setRollerSpeed(-0.1)
        SmartDashboard.putNumber("Note Timer", timer.get())
        if(!launcherSubsystem.noteTrigger.asBoolean){
            timer.reset()
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.15) && launcherSubsystem.noteTrigger.asBoolean
    }

    override fun end(interrupted: Boolean) {
        println("Intake note command stopped, Interrupted: $interrupted")
        timer.stop()
        launcherSubsystem.setRollerPosition(
            launcherSubsystem.getRollerPosition() + 0.3
        )
    }
}
