package frc.robot.commands.pivot

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem
import lib.math.units.Rotation
import java.util.function.Supplier

/**
 * My attempt at a proof-of-concept
 */
class ContinuousPivotCommand(private val changer: Supplier<Rotation>) : Command() {
    private val current: MutableMeasure<Angle> = Degrees.zero().mutableCopy()

    init {
        addRequirements(PivotSubsystem)
    }

    override fun execute() {
        PivotSubsystem.tryUpdate()

        val change = changer.get()
        if(change.magnitude() == 0.0)
            return

        current.mut_plus(change)
        PivotSubsystem.targetPosition = current
    }
}