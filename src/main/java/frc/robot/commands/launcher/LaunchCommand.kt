package frc.robot.commands.launcher

import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LauncherSubsystem
import lib.math.units.inMPS
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class LaunchCommand(
    private val subsystem: LauncherSubsystem,
    private val launch: BooleanSupplier,
    private val shouldAmp: BooleanSupplier
) : Command() {
    private val timer = Timer()
    private val minimumVelocity: Measure<Velocity<Distance>> = 23.inMPS
    private var lastShouldAmp = false

    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        subsystem.setFlywheelBrakes(false)
        timer.restart()

        if(!shouldAmp.asBoolean) {
            subsystem.setFlywheelVelocity(25.inMPS)
        } else {
            subsystem.setFlywheelVelocity(8.inMPS)
        }
    }

    override fun execute() {
        if (shouldAmp.asBoolean != !lastShouldAmp) {
            if(!shouldAmp.asBoolean) {
                subsystem.setFlywheelVelocity(25.inMPS)
            } else {
                subsystem.setFlywheelVelocity(8.inMPS)
            }
        }

        if (subsystem.noteDetected) {
            timer.reset()
        }

        if (
            launch.asBoolean &&
            subsystem.getTopFlywheelVelocity() >= minimumVelocity &&
            subsystem.getBottomFlywheelVelocity() >= minimumVelocity
        ) {
            subsystem.setRawRollerVoltage(-6.0)
        }
    }

}