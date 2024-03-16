package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem
import java.util.function.DoubleSupplier

class TestFlywheelSpeedsCommand(speed: DoubleSupplier) : Command() {
    private val launcherSubsystem = LauncherSubsystem
    private val speed: DoubleSupplier


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
        this.speed = speed
    }

    override fun initialize() {}

    override fun execute() {
        launcherSubsystem.setFlywheelSpeeds(speed.asDouble)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
