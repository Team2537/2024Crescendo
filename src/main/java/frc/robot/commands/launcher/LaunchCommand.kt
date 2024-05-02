package frc.robot.commands.launcher

import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.launcher.LaunchSubsystem
import frc.robot.subsystems.pivot.PivotIO
import lib.math.units.degrees
import lib.math.units.inchesPerSecond
import lib.math.units.into
import java.util.function.BooleanSupplier

class LaunchCommand(
    private val targetSpeed: Measure<Velocity<Distance>> = 925.inchesPerSecond,
    private val shouldLaunch: BooleanSupplier
) : Command() {
    private val subsystem = LaunchSubsystem
    private val io = subsystem.io
    private val inputs = subsystem.input
    private val timer: Timer = Timer()
    private val minVelocity: Measure<Velocity<Distance>> = 900.inchesPerSecond
    private val pivotInputs: PivotIO.PivotIOInputs = PivotIO.PivotIOInputs() // Fix this later
    private val feedforward_t: SimpleMotorFeedforward = subsystem.topFlywheelFeedforward
    private val feedforward_b: SimpleMotorFeedforward = subsystem.bottomFlywheelFeedforward
    private var shotType: LaunchSubsystem.LaunchType = LaunchSubsystem.LaunchType.SPEAKER


    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        io.setFlywheelsBrakeMode(false)
        timer.restart()
    }

    override fun execute() {
        if (!pivotInputs.relativeAngle.gt(75.0.degrees)) {
            shotType = LaunchSubsystem.LaunchType.SPEAKER
            io.runSetpoint(
                targetSpeed,
                feedforward_t.calculate(targetSpeed.into(Units.InchesPerSecond)),
                feedforward_b.calculate(targetSpeed.into(Units.InchesPerSecond)),
                SparkPIDController.ArbFFUnits.kVoltage
            )
        } else {
            shotType = LaunchSubsystem.LaunchType.AMP
            io.runSetpoint(
                targetSpeed.divide(4.0),
                feedforward_t.calculate(targetSpeed.divide(4.0).into(Units.InchesPerSecond)),
                feedforward_b.calculate(targetSpeed.divide(4.0).into(Units.InchesPerSecond)),
                SparkPIDController.ArbFFUnits.kVoltage
            )
        }

        if (inputs.rightNoteDetected) timer.reset()

        if (shouldLaunch.asBoolean && inputs.rightNoteDetected) {
            if (shotType == LaunchSubsystem.LaunchType.AMP
                && inputs.topFlywheelsVelocity.gte(minVelocity.divide(4.0))
            ) {
                io.setRollerVoltage(Units.Volts.of(-12.0))
            } else if (inputs.topFlywheelsVelocity.gte(minVelocity)
                && inputs.bottomFlywheelsVelocity.gte(minVelocity)
            ) {
                io.setRollerVoltage(Units.Volts.of(-12.0))
            }
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.5)
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        io.stopFlywheels()
        io.setRollerBrakeMode(true)
        io.stopRoller()
        io.setFlywheelsBrakeMode(true)
    }


}