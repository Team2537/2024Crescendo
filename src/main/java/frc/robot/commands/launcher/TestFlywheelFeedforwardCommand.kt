package frc.robot.commands.launcher

import LauncherSubsystem
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import lib.math.units.rpm

class TestFlywheelFeedforwardCommand : Command() {
    val launcherSubsystem = LauncherSubsystem

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    override fun initialize() {
        launcherSubsystem.setFlywheelVelocity(Units.RPM.of(5000.0))
        println(launcherSubsystem.topFlywheelFeedforward.calculate(83.0))
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
