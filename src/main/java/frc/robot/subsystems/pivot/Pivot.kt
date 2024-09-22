package frc.robot.subsystems.pivot

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
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
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        )

        Constants.RobotConstants.Mode.SIM -> PivotIOSim(
            DCMotor.getNEO(1),
            GEARBOX_RATIO * PULLEY_RATIO,
            0.26,
            Units.Inches.of(19.6),
            Units.Degrees.of(90.0),
            Units.Degrees.of(0.0),
            Units.Degrees.of(90.0),
            10.0, 0.5, 0.0,
            0.0, 0.43, 1.34, 0.02
        )

        Constants.RobotConstants.Mode.REPLAY -> object : PivotIO {}
    }

    private val inputs = PivotIO.PivotInputs()

    val mechanism: Mechanism2d = Mechanism2d(3.0, 3.0)
    val root = mechanism.getRoot("pivot", 2.0, 1.0)
    val arm = root.append(MechanismLigament2d("arm", 0.5, 90.0))

    fun manualControl(voltage: DoubleSupplier) = run {
        io.setRawVoltage(
            Units.Volts.of(voltage.asDouble * 3),
            false
        )
    }

    fun home() = run { io.setRawVoltage(Units.Volts.of(-3.0), false) }.until(inputs::isAtHardstop)
        .andThen(run { io.setKnownPosition(Units.Degrees.of(90.0)) })

    fun sendToPosition(position: Measure<Angle>) = run { io.setTargetPosition(position) }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("pivot", inputs)

        arm.angle = inputs.relativePosition into Units.Degrees

//        io.setRawVoltage(Units.Volts.of(0.0), false)

        Logger.recordOutput("pivot/mechanism", mechanism)
    }

    companion object {
        const val GEARBOX_RATIO: Double = 36.0 / 1.0
        const val PULLEY_RATIO: Double = 40.0 / 18.0

        const val ABSOLUTE_ENCODER_PORT = 1
        const val PIVOT_MOTOR_PORT = 16
        const val HOMING_SENSOR_PORT = 2

        const val INVERT = false
    }
}