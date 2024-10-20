package frc.robot.subsystems.intake

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.Robot
import lib.math.EdgeDetector
import lib.math.units.feet
import lib.toTransform2d
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

/**
 * The Intake subsystem contains command factories for the intake.
 */
class Intake : SubsystemBase() {
    private val io: IntakeIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> IntakeIONeo(
            19,
            8,
            9,
            rollerDiameter,
            1.0
        )

        Constants.RobotConstants.Mode.SIM -> IntakeIOSim(
            1.0,
            rollerMOI,
            rollerDiameter,
            { state }
        )

        Constants.RobotConstants.Mode.REPLAY -> object : IntakeIO {}
    }

    private val inputs: IntakeIO.IntakeInputs = IntakeIO.IntakeInputs()

    private val exitEdgeDetector = EdgeDetector()
    private val intakeEdgeDetector = EdgeDetector()

    private var state: IntakeState = IntakeState.EMPTY
    val isFull: Trigger = Trigger { state == IntakeState.STORED }

    fun getMoveNoteCommand(
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

    fun getIntakeCommand() = getMoveNoteCommand(Direction.IN, Sensor.INTAKE, IntakeState.STORED).withName("Intake")

    fun getEjectCommand() = getMoveNoteCommand(Direction.OUT, Sensor.INTAKE, IntakeState.EMPTY).withName("Eject")

    fun getTransferCommand() = getMoveNoteCommand(Direction.IN, Sensor.EXIT, IntakeState.EMPTY).withName("Transfer")

    fun getSimpleIntakeCommand() =
        Commands.sequence(
            runOnce {io.setVoltage(Volts.of(9.0))},
            Commands.waitSeconds(4.5),
            runOnce { io.stop(); state = IntakeState.STORED }
        ).withName("Simple Timed Intake")

    fun getConstantIntakeCommand() = runEnd(
        { io.setVoltage(Volts.of(9.0)) },
        {io.stop(); state = IntakeState.STORED}
    )

    fun getStopCommand() = runOnce {io.stop()}

    fun getManualIntakeCommand(volts: DoubleSupplier): Command {
        return run {io.setVoltage(Volts.of(volts.asDouble))}
    }

    override fun periodic() {
        exitEdgeDetector.update(inputs.exitSensorTriggered)
        intakeEdgeDetector.update(inputs.intakeSensorTriggered)

        io.updateInputs(inputs)
        Logger.processInputs("intake", inputs)

        Logger.recordOutput("intake/state", state)
        Logger.recordOutput("intake/intakePoint", Pose2d.struct, Robot.robotPose.transformBy(poseToIntakePoint.toTransform2d()))
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

        val poseToIntakePoint = Transform3d(Translation3d((14.0 / 12).feet, 0.0.feet, 0.0.feet), Rotation3d())
    }
}
