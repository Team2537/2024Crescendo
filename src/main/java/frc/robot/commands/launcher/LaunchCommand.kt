package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem
import edu.wpi.first.wpilibj.Timer
import lib.math.units.velocity
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class LaunchCommand(
    speed: DoubleSupplier,
    launch: BooleanSupplier,
    pivotAngle: DoubleSupplier
    ) : Command() {
    private val launcherSubsystem = LauncherSubsystem
    private val speed: DoubleSupplier
    private val timer: Timer = Timer()
    private val launch: BooleanSupplier
    private val pivotAngle: DoubleSupplier
    private var minVelocity: Double = 6000.0

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
        this.speed = speed
        this.launch = launch
        this.pivotAngle = pivotAngle
    }

    override fun initialize() {
        timer.restart()
        println("Starting Launch")
    }

    override fun execute() {
        if(pivotAngle.asDouble < 10) {
            launcherSubsystem.setFlywheelSpeeds(speed.asDouble / 4)
            minVelocity = 1200.0
        } else {
            launcherSubsystem.setFlywheelSpeeds(speed.asDouble)
            minVelocity = 6000.0

        }
        if(launcherSubsystem.noteTrigger.asBoolean){
            timer.reset()
        }

        if(launcherSubsystem.noteTrigger.asBoolean
            && launch.asBoolean
            && launcherSubsystem.topFlywheels.encoder.velocity > minVelocity){
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
