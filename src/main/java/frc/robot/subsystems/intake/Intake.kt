package frc.robot.subsystems.intake

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.Robot
import lib.math.EdgeDetector
import org.littletonrobotics.junction.Logger

/**
 * The Intake subsystem contains command factories for the intake.
 */
class Intake : SubsystemBase() {
    private val io: IntakeIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> IntakeIONeo(
            100,
            101,
            102,
            rollerDiameter,
            1.0
        )

        Constants.RobotConstants.Mode.SIM -> IntakeIOSim(
            1.0,
            rollerMOI,
            rollerDiameter,
            { Robot.keyboard.getRawButton(1) },
            { Robot.keyboard.getRawButton(2) }
        )

        Constants.RobotConstants.Mode.REPLAY -> object : IntakeIO {}
    }

    private val inputs: IntakeIO.IntakeInputs = IntakeIO.IntakeInputs()

    private val exitEdgeDetector = EdgeDetector()
    private val intakeEdgeDetector = EdgeDetector()

    private var state: IntakeState = IntakeState.EMPTY
    val isFull: Trigger = Trigger { state == IntakeState.STORED }

    fun moveNote(
        direction: Direction,
        sensor: Sensor,
        endState: IntakeState,
        voltageMagnitude: Double = 12.0
    ): Command {
        return runOnce {
            io.setVoltage(
                when (direction) {
                    Direction.IN -> Units.Volts.of(voltageMagnitude)
                    Direction.OUT -> Units.Volts.of(-voltageMagnitude)
                }
            )
        }.andThen(
            WaitUntilCommand {
                when (sensor) {
                    Sensor.INTAKE -> intakeEdgeDetector.fallingEdge
                    Sensor.EXIT -> exitEdgeDetector.fallingEdge
                }
            }
        ).andThen(runOnce { io.stop(); state = endState })

    }

    fun intake() = moveNote(Direction.IN, Sensor.INTAKE, IntakeState.STORED)

    fun eject() = moveNote(Direction.OUT, Sensor.INTAKE, IntakeState.EMPTY)

    fun transfer() = moveNote(Direction.IN, Sensor.EXIT, IntakeState.EMPTY)

    override fun periodic() {
        exitEdgeDetector.update(inputs.exitSensorTriggered)
        intakeEdgeDetector.update(inputs.intakeSensorTriggered)

        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    enum class IntakeState {
        EMPTY,
        STORED
    }

    enum class Direction {
        IN,
        OUT
    }

    enum class Sensor {
        INTAKE,
        EXIT
    }

    companion object {
        val rollerDiameter = Units.Inches.of(0.84)
        val rollerMOI = 0.0000719
    }
}
