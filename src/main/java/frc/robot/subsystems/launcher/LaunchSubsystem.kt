package frc.robot.subsystems.launcher

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger

object LaunchSubsystem : SubsystemBase() {
    var io: LauncherIO? = null
    val input: LauncherIO.LauncherIOInputs = LauncherIO.LauncherIOInputs()

    /** Feedforward controller for the top flywheel */
    val topFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(-0.20832, 0.11109, 0.024896)

    /** Feedforward controller for the bottom flywheel */
    val bottomFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.035079, 0.10631, 0.0080339)

    val topSysIDRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null, null, null,
            {state -> Logger.recordOutput("launcher/SysID/top", state.toString())}
        ),
        SysIdRoutine.Mechanism(
            { voltage: Measure<Voltage> -> io?.setFlywheelVoltage(voltage, Units.Volts.zero()) },
            null,
            this
        )
    )

    val bottomSysIDRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null, null, null,
            {state -> Logger.recordOutput("launcher/SysID/bottom", state.toString())}
        ),
        SysIdRoutine.Mechanism(
            { voltage: Measure<Voltage> -> io?.setFlywheelVoltage(Units.Volts.zero(), voltage) },
            null,
            this
        )
    )

    init {
        io = LauncherIONeos()
    }

    override fun periodic() {
        io?.updateInputs(input)
        Logger.processInputs("Launcher", input)
    }

    enum class LaunchType {
        AMP, SPEAKER
    }
}