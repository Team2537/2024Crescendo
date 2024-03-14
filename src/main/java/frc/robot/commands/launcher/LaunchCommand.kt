package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem
import edu.wpi.first.wpilibj.Timer
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class LaunchCommand(
    speed: DoubleSupplier,
    launch: BooleanSupplier,
    minimumVelocity: Double = 1000.0
    ) : Command() {
    private val launcherSubsystem = LauncherSubsystem
    private val speed: DoubleSupplier
    private val timer: Timer = Timer()
    private val launch: BooleanSupplier
    private val minimumVelocity: Double

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
        this.speed = speed
        this.launch = launch
        this.minimumVelocity = minimumVelocity
    }

    override fun initialize() {
        timer.restart()
        println("Starting Launch")
    }

    override fun execute() {
        launcherSubsystem.setFlywheelSpeeds(speed.asDouble)
        if(launcherSubsystem.noteTrigger.asBoolean){
            timer.reset()
        }

        if(launcherSubsystem.noteTrigger.asBoolean
            && launch.asBoolean
            && launcherSubsystem.topFlywheels.encoder.velocity > minimumVelocity){
            launcherSubsystem.setRollerSpeed(-1.0)
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.25)
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        launcherSubsystem.stopFlywheels()
        launcherSubsystem.stopRoller()
        println("Launch Ending $interrupted")
    }
}
