package frc.robot.subsystems.pivot

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.Constants
import lib.ControllerGains
import lib.math.units.degrees
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

class Pivot : SubsystemBase() {
    private val io: PivotIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> PivotIONeos(
            PIVOT_MOTOR_PORT,
            INVERT,
            ABSOLUTE_ENCODER_PORT,
            GEARBOX_RATIO * PULLEY_RATIO,
            PULLEY_RATIO,
            HOMING_SENSOR_PORT,
            ControllerGains()
        )

        Constants.RobotConstants.Mode.SIM -> PivotIOSim(
            DCMotor.getNEO(1),
            GEARBOX_RATIO * PULLEY_RATIO,
            0.26,
            Units.Inches.of(19.6),
            Units.Degrees.of(90.0),
            Units.Degrees.of(0.0),
            Units.Degrees.of(90.0),
            ControllerGains(
                kP = 10.0, kI = 0.5, kD = 0.0,
                kS = 0.0, kG = 0.43, kV = 1.34, kA = 0.02
            )
        )

        Constants.RobotConstants.Mode.REPLAY -> object : PivotIO {}
    }

    private val inputs = PivotIO.PivotInputs()

    private val setpoint: MutableMeasure<Angle> = MutableMeasure.zero(Degrees)

    val mechanism: Mechanism2d = Mechanism2d(3.0, 3.0)
    val root = mechanism.getRoot("pivot", 2.0, 1.0)
    val arm = root.append(MechanismLigament2d("arm", 0.5, 90.0))

    private val currentSpikeTracker = LinearFilter.highPass(5.0, 0.02)

    fun manualControl(voltage: DoubleSupplier) = run {
        io.setRawVoltage(
            Units.Volts.of(voltage.asDouble),
            false
        )
    }.withName("Manual Control")

    fun getHomeCommand() =
        runOnce { io.setRawVoltage(Units.Volts.of(-3.0)) }
            .andThen(WaitUntilCommand { inputs.isAtHardstop })
            .andThen(runOnce { io.setKnownPosition(Units.Degrees.of(90.0)); io.stop() }).withName("Home Pivot")

    fun getSensorlessHomeCommand(threshold: Double = 5.5) =
        runOnce { io.setRawVoltage(Units.Volts.of(-3.0)) }
            .andThen(WaitUntilCommand { currentSpikeTracker.lastValue() > threshold })
            .andThen(runOnce { io.setKnownPosition(Units.Degrees.of(90.0)); io.stop() })
            .withName("Sensorless Home Pivot")

    fun getSendToPositionCommand(position: Measure<Angle>) =
        runOnce { io.setTargetPosition(position) }
            .until { inputs.relativePosition.isNear(position, 1.0)}
            .withName("Send To Position: ${position.baseUnitMagnitude()}")

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("pivot", inputs)

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
    }

    companion object {
        const val GEARBOX_RATIO: Double = 36.0 / 1.0
        const val PULLEY_RATIO: Double = 40.0 / 18.0

        const val ABSOLUTE_ENCODER_PORT = 1
        const val PIVOT_MOTOR_PORT = 16
        const val HOMING_SENSOR_PORT = 3

        const val INVERT = false

        val originToPivot = Translation3d(
            0.0,
            0.0,
            0.516847 + 0.042919
        )

        val intakePosition = 16.5.degrees
    }
}