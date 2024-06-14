package frc.robot.commands.pivot

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PivotSubsystem
import lib.math.units.inRPS
import lib.math.units.inRadians
import lib.math.units.into

class AutoPivotCommand(
    private val subsystem: PivotSubsystem,
    private val setpoint: Double
) : Command() {
    private lateinit var startState: TrapezoidProfile.State
    private lateinit var endState: TrapezoidProfile.State
    private lateinit var targetState: TrapezoidProfile.State
    private lateinit var profile: TrapezoidProfile
    private var timer: Timer = Timer()

    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        startState = TrapezoidProfile.State(
            subsystem.getAngle().into(Units.Radians),
            subsystem.getVelocity().into(Units.RadiansPerSecond)
        )

        endState = TrapezoidProfile.State(
            setpoint,
            0.0
        )

        timer.restart()
    }

    override fun execute() {
        val elapsed = timer.get()
        targetState = if(profile.isFinished(elapsed)){
            endState
        } else {
            profile.calculate(elapsed, startState, endState)
        }

        val feedforward = subsystem.calculateFeedforward(subsystem.getAngle(), targetState.velocity.inRPS)
        subsystem.applyPID(targetState.position.inRadians, feedforward)
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
        timer.stop()
    }
}