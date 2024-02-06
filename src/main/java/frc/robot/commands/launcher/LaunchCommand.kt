package frc.robot.commands.launcher

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LauncherSubsystem

class LaunchCommand : Command() {
    private val launcherSubsystem = LauncherSubsystem

    private val timer: Timer

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)

        timer = Timer()
    }

    override fun initialize() {
        launcherSubsystem.setLaunchVelocity(6700.0)
        timer.restart()
    }

    override fun execute() {
        if(launcherSubsystem.leftLauncherMotor.encoder.velocity > 6600.0){
            launcherSubsystem.outtake()
        }

        if(launcherSubsystem.piecePresent){
            timer.reset()
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.3)
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
    }
}
