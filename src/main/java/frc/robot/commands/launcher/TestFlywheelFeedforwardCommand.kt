package frc.robot.commands.launcher

import LauncherSubsystem
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import lib.math.units.rpm

/**
 * Command to test the feedforward velocity control of the flywheel
 */
class TestFlywheelFeedforwardCommand : Command() {
    /** The subsystem that this command runs on. */
    val launcherSubsystem = LauncherSubsystem

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
    }

    /**
     * Set the flywheel to a constant velocity to test the feedforward control
     */
    override fun initialize() {
        launcherSubsystem.setFlywheelVelocity(Units.RPM.of(5000.0))
        println(launcherSubsystem.topFlywheelFeedforward.calculate(83.0))
    }

    /**
     * This command is never finished on its own.
     * It will be finished when the command is interrupted or canceled.
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }
}
