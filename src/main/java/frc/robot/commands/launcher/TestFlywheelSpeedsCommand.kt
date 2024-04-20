package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem
import java.util.function.DoubleSupplier

/**
 * Command to test the flywheel speeds manually
 * @param speed a supplier that returns the desired speed of the flywheels
 */
class TestFlywheelSpeedsCommand(speed: DoubleSupplier) : Command() {
    /** The subsystem that this command runs on. */
    private val launcherSubsystem = LauncherSubsystem
    /** The supplier that returns the desired speed of the flywheels */
    private val speed: DoubleSupplier


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
        this.speed = speed
    }

    /**
     * Set the flywheels to the desired speed
     * Runs every 20ms
     */
    override fun execute() {
        launcherSubsystem.setFlywheelSpeeds(speed.asDouble)
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
