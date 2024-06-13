package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import LauncherSubsystem
import com.revrobotics.CANSparkBase
import edu.wpi.first.wpilibj.Timer
import lib.math.units.inRPM
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * Command to launch the note
 * @param speed a supplier that returns the desired speed of the flywheels
 * @param launch a supplier that returns whether the note should be launched
 * @param pivotAngle a supplier that returns the angle of the pivot
 * @param override a supplier that returns whether the launch should be overridden
 */
class LaunchCommand(
    speed: DoubleSupplier,
    launch: BooleanSupplier,
    pivotAngle: DoubleSupplier,
    override: BooleanSupplier
    ) : Command() {
    /** The subsystem that this command runs on. */
    private val launcherSubsystem = LauncherSubsystem
    /** The supplier that returns the desired speed of the flywheels */
    private val speed: DoubleSupplier
    /** Timer to keep track of how long the note has been outside the launcher */
    private val timer: Timer = Timer()
    /** The supplier that returns whether the note should be launched */
    private val launch: BooleanSupplier
    /** The supplier that returns the angle of the pivot */
    private val pivotAngle: DoubleSupplier
    /** The minimum velocity that the flywheels should be at before the note is launched */
    private var minVelocity: Double = 6000.0
    /** The supplier that returns whether the launch should be overridden */
    private var override: BooleanSupplier

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(launcherSubsystem)
        this.speed = speed
        this.launch = launch
        this.pivotAngle = pivotAngle
        this.override = override
    }

    /**
     * Set the flywheels to coast mode so they have less resistance on the note.
     * Reset the timer so it starts counting from 0 when the command is initialized.
     */
    override fun initialize() {
        launcherSubsystem.setFlywheelBrake(false)
        timer.restart()
        println("Starting Launch")
    }

    /**
     * Set the flywheels to the desired speed and the roller motor to push the note through the launcher.
     * If the note trigger is pressed and the flywheels are at the desired speed, launch the note.
     * Runs every 20ms
     */
    override fun execute() {
        // If the pivot angle is less than 10 degrees, set the flywheels to a quarter of the desired speed
        // This is for amp shots
        if(pivotAngle.asDouble < 10) {
            launcherSubsystem.setFlywheelSpeeds(speed.asDouble / 4)
            minVelocity = 1200.0
        } else {
            launcherSubsystem.setFlywheelVelocity(6100.0.inRPM)
            minVelocity = 6000.0

        }

        // If the note trigger is pressed, reset the timer so it starts counting from 0
        // This is used to make sure the flywheels dont end before the note is launched
        if(launcherSubsystem.noteTrigger.asBoolean){
            timer.reset()
        }

        // If the note trigger is pressed and the flywheels are at the desired speed, launch the note
        // If the override is pressed, launch the note
        if(launcherSubsystem.noteTrigger.asBoolean
            && ((launch.asBoolean
            && launcherSubsystem.topFlywheels.encoder.velocity > minVelocity) || override.asBoolean)){
            launcherSubsystem.setRollerSpeed(-1.0)
        }
    }

    /**
     * The command is finished when the note has been launched for 0.5 seconds
     */
    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.5)
    }

    /**
     * Stop the flywheels and the roller motor when the command is interrupted or canceled
     */
    override fun end(interrupted: Boolean) {
        // Stop the flywheels and the roller motor
        timer.stop()
        // Stop and brake the flywheels and stop the roller motor
        launcherSubsystem.stopFlywheels()
        launcherSubsystem.rollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        launcherSubsystem.stopRoller()
        launcherSubsystem.setFlywheelBrake(true)
        println("Launch Ending $interrupted")
    }
}
