package frc.robot.subsystems.superstructure.pivot

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants
import lib.ControllerGains
import lib.LoggedTunableNumber
import lib.math.units.degrees
import lib.math.units.degreesPerSecond
import lib.math.units.into
import lib.math.units.unaryMinus
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

class Pivot : SubsystemBase() {
    private val kP = LoggedTunableNumber("pivot/kP", 0.5)
    private val kI = LoggedTunableNumber("pivot/kI", 0.0)
    private val kD = LoggedTunableNumber("pivot/kD", 0.0)

    private val io: PivotIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> PivotIONeos(
            PIVOT_MOTOR_PORT,
            INVERT,
            ABSOLUTE_ENCODER_PORT,
            GEARBOX_RATIO * PULLEY_RATIO,
            PULLEY_RATIO,
            HOMING_SENSOR_PORT,
            ControllerGains(
                kP = kP.get(), kI = kI.get(), kD = kD.get(),
                kG = 0.39
            )
        )

        Constants.RobotConstants.Mode.SIM -> PivotIOSim(
            DCMotor.getNEO(1),
            GEARBOX_RATIO * PULLEY_RATIO,
            0.26,
            Units.Inches.of(19.6),
            Degrees.of(90.0),
            Degrees.of(0.0),
            Degrees.of(90.0),
            ControllerGains(
                kP = 10.0, kI = 0.5, kD = 0.0,
                kS = 0.0, kG = 0.43, kV = 1.34, kA = 0.02
            )
        )

        Constants.RobotConstants.Mode.REPLAY -> object : PivotIO {}
    }.apply { setKnownPosition(Degrees.of(0.0)) }

    private val inputs = PivotIO.PivotInputs()

    private val setpoint: MutableMeasure<Angle> = MutableMeasure.zero(Degrees)

    val mechanism: Mechanism2d = Mechanism2d(3.0, 3.0)
    val root = mechanism.getRoot("pivot", 2.0, 1.0)
    val arm = root.append(MechanismLigament2d("arm", 0.5, 90.0))

    private val currentSpikeTracker = LinearFilter.highPass(0.2, 0.02)

    private val isStalled: Trigger =
        Trigger { currentSpikeTracker.lastValue() > 5.5 && inputs.velocity <= 5.0.degreesPerSecond }

    private val sysIdRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Volts.of(5.0),
            Seconds.of(3.0),
            { state -> Logger.recordOutput("pivot/sysidState", state.name) }
        ),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> -> io.setRawVoltage(volts) },
            null,
            this
        )
    )

    fun getSysIDRoutine() =
        Commands.sequence(
            getDynamicSysID(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(2.0),
            getDynamicSysID(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(5.0),
            getQuasistaticSysID(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(5.0),
            getQuasistaticSysID(SysIdRoutine.Direction.kReverse)
        ).finallyDo { interupt -> println("ending sysid, interupt: $interupt") }

    fun manualControl(voltage: DoubleSupplier) = run {
        io.setRawVoltage(
            Volts.of(voltage.asDouble),
            false
        )
    }.withName("Manual Control")

    fun getHomeCommand() =
        Commands.sequence(
            runOnce { io.setRawVoltage(Volts.of(-3.0)) },
            Commands.waitUntil { inputs.isAtHardstop },
            runOnce { io.setKnownPosition(Units.Degrees.of(90.0)); io.stop() }
        ).withName("Home Pivot")

    fun getQuasistaticSysID(direction: SysIdRoutine.Direction): Command {
        return Commands.sequence(
            PrintCommand("Starting Quasistatic ${direction.name}"),
            Commands.waitSeconds(1.0),
            Commands.waitUntil {
                when (direction) {
                    SysIdRoutine.Direction.kForward -> inputs.velocity <= 0.5.degreesPerSecond
                    SysIdRoutine.Direction.kReverse -> inputs.velocity >= -0.5.degreesPerSecond
                }
            },
            Commands.waitSeconds(0.25),
            PrintCommand("Ending Quasistatic ${direction.name}")
        ).deadlineWith(sysIdRoutine.quasistatic(direction))
    }

    fun getDynamicSysID(direction: SysIdRoutine.Direction): Command {
        return Commands.sequence(
            PrintCommand("Starting dynamic ${direction.name}"),
            Commands.waitSeconds(0.25),
            Commands.waitUntil(isStalled),
            Commands.waitSeconds(0.25),
            PrintCommand("Ending dynamic ${direction.name}")
        ).deadlineWith(sysIdRoutine.dynamic(direction))
    }


    fun getSensorlessHomeCommand() =
        Commands.sequence(
            runOnce { io.setRawVoltage(Volts.of(-3.0)) },
            Commands.waitSeconds(0.25),
            Commands.waitUntil(isStalled),
            runOnce { io.stop() },
            Commands.waitSeconds(0.25),
            getResetPositionCommand()
        ).withName("Sensorless Home Pivot")

    fun getQuickAngleCommand(position: Measure<Angle>): Command {
        val direction = position > inputs.relativePosition

        return Commands.sequence(
            Commands.either(
                runOnce { io.setRawVoltage(Volts.of(3.0)) },
                runOnce { io.setRawVoltage(Volts.of(-3.0)) },
                { direction }
            ),
            Commands.either(
                Commands.waitUntil { inputs.relativePosition >= position },
                Commands.waitUntil { inputs.relativePosition <= position },
                { direction }
            ),
            runOnce { io.setTargetPosition(position) }
        )
    }

    fun getResetPositionCommand() = runOnce { io.setKnownPosition(0.0.degrees) }

    fun getSendToPositionCommand(position: Measure<Angle>) =
        Commands.sequence(
            runOnce { io.setTargetPosition(position) },
            Commands.waitUntil { inputs.relativePosition.isNear(position, 0.1) }
                .alongWith(PrintCommand("pivoting...").repeatedly())
        ).withName("Send To Position: ${position.baseUnitMagnitude()}")

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("pivot", inputs)
        Logger.recordOutput("isStalled", isStalled.asBoolean)

        arm.angle = inputs.relativePosition into Units.Degrees

//        io.setRawVoltage(Units.Volts.of(0.0), false)

        Logger.recordOutput(
            "pivot/mechanismPose",
            Pose3d.struct,
            Pose3d(
                originToPivot,
                Rotation3d(0.0, -(inputs.relativePosition into Units.Radians), 0.0)
            )
        )

        Logger.recordOutput("pivot/mechanism2d", mechanism)

        Logger.recordOutput("pivot/currentSpike", currentSpikeTracker.calculate(inputs.appliedCurrent into Units.Amps))

        LoggedTunableNumber.ifChanged(hashCode(), { pid ->
            io.setPID(pid[0], pid[1], pid[2])
        }, kP, kI, kD)
    }

    companion object {
        const val GEARBOX_RATIO: Double = 36.0 / 1.0
        const val PULLEY_RATIO: Double = 40.0 / 18.0

        const val ABSOLUTE_ENCODER_PORT = 1
        const val PIVOT_MOTOR_PORT = 16
        const val HOMING_SENSOR_PORT = 3

        const val INVERT = true

        val originToPivot = Translation3d(
            -0.242022,
            0.05,
            0.516847 + 0.042919
        )
    }
}