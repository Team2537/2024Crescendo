package frc.robot.commands.pivot

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.pivot.PivotSubsystem
import frc.robot.subsystems.pivot.PivotSubsystem.PivotDirection
import lib.math.units.into

class QuickPivotCommand(
    private val target: Measure<Angle>
) : Command() {
    private val subsystem = PivotSubsystem
    private val io = subsystem.io
    private val inputs = subsystem.inputs


    private var direction: PivotDirection = PivotDirection.UP

    init {
        addRequirements(PivotSubsystem)
    }

    override fun initialize() {
        if (inputs.relativeAngle > target) {
            direction = PivotDirection.DOWN
        } else {
            direction = PivotDirection.UP
        }
    }

    override fun execute() {
        if (inputs.relativeAngle > target) {
            io.setRawVoltage(Units.Volts.of(-0.2 * 12))
        } else {
            io.setRawVoltage(Units.Volts.of(0.2 * 12))
        }
    }

    override fun isFinished(): Boolean {
        return when (direction) {
            PivotDirection.UP -> inputs.relativeAngle >= target
            PivotDirection.DOWN -> inputs.relativeAngle <= target
        }
    }

    override fun end(interrupted: Boolean) {
        io.runSetpoint(
            target,
            Units.Volts.of(
                subsystem.feedforward.calculate(
                    target.into(Units.Radians),
                    0.0
                )
            )
        )
    }
}