package frc.robot.subsystems.climb

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import kotlin.math.sin
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.BooleanSupplier

class Climb : SubsystemBase() {
    private val leftArmIO: ClimberArmIO = when(Constants.RobotConstants.mode){
        Constants.RobotConstants.Mode.REAL -> ClimberArmNeo(
            leftID,
            true,
            gearing,
            drumRadius
        )
        Constants.RobotConstants.Mode.SIM -> ClimberArmIOSim(
            DCMotor.getNEO(1),
            gearing, load, drumRadius, maxExtension
        )
        Constants.RobotConstants.Mode.REPLAY -> object : ClimberArmIO {}
    }

    private val rightArmIO: ClimberArmIO = when(Constants.RobotConstants.mode){
        Constants.RobotConstants.Mode.REAL -> ClimberArmNeo(
            rightID,
            false,
            gearing,
            drumRadius
        )
        Constants.RobotConstants.Mode.SIM -> ClimberArmIOSim(
            DCMotor.getNEO(1),
            gearing, load, drumRadius, maxExtension
        )
        Constants.RobotConstants.Mode.REPLAY -> object : ClimberArmIO {}
    }

    val leftInputs: ClimberArmIO.ClimberArmInputs = ClimberArmIO.ClimberArmInputs()
    val rightInputs: ClimberArmIO.ClimberArmInputs = ClimberArmIO.ClimberArmInputs()

    var state = ClimbState.PRECLIMB
        private set

    val isPreclimb: Trigger = Trigger { state == ClimbState.PRECLIMB }
    val isExtended: Trigger = Trigger { state == ClimbState.EXTENDED }
    val isLatched: Trigger = Trigger { state == ClimbState.LATCHED }

    override fun periodic() {
        leftArmIO.updateInputs(leftInputs)
        rightArmIO.updateInputs(rightInputs)

        Logger.processInputs("climb/leftArm", leftInputs)
        Logger.processInputs("climb/rightArm", rightInputs)

        Logger.recordOutput("climb/leftArmPose", Pose3d.struct, Pose3d(
            Translation3d(
                0.0,
                Units.inchesToMeters(10.0),
                (leftInputs.relativePosition into Meters) + Units.inchesToMeters(8.0)
            ),
            Rotation3d()
        ))

        Logger.recordOutput("climb/rightArmPose", Pose3d.struct, Pose3d(
            Translation3d(
                0.0,
                Units.inchesToMeters(-10.0),
                (rightInputs.relativePosition into Meters) + Units.inchesToMeters(8.0)
            ),
            Rotation3d()
        ))

        Logger.recordOutput("climb/state", state)
    }

    fun getSineCommand() = run {
        val voltage = sin(Timer.getFPGATimestamp()) * 12.0
        leftArmIO.setVoltage(Volts.of(voltage), false)
        rightArmIO.setVoltage(Volts.of(voltage), false)
    }

    fun getRespoolCommand(): Command {
        return runEnd(
            {
                leftArmIO.setVoltage(Volts.of(-3.0))
                rightArmIO.setVoltage(Volts.of(-3.0))
            },
            {
                leftArmIO.stop()
                rightArmIO.stop()
            }
        )
    }

    fun getExtendCommand() = sequence(
        runOnce {
            leftArmIO.setVoltage(extensionVoltage, false)
            rightArmIO.setVoltage(extensionVoltage, false)
        },
        waitSeconds(1.5),
        runOnce {
            leftArmIO.stop()
            rightArmIO.stop()
            state = ClimbState.EXTENDED
        }
    )

    fun getRetractCommand() = sequence(
        runOnce {
            leftArmIO.setVoltage(retractionVoltage, false)
            rightArmIO.setVoltage(retractionVoltage, false)
        },
        waitUntil {
            leftInputs.appliedCurrent > currentThreshold || rightInputs.appliedCurrent > currentThreshold
        },
        runOnce {
            leftArmIO.stop()
            rightArmIO.stop()
            state = ClimbState.LATCHED
        }
    )

    enum class ClimbState {
        PRECLIMB,
        EXTENDED,
        LATCHED
    }

    companion object {
        val leftID = 15
        val rightID = 18

        val maxExtension = Inches.of(48.0)
        val drumRadius = Inches.of(0.325)

        val load = Constants.RobotConstants.robotWeight.divide(2.0)
        val gearing = 16/1.0

        val extensionVoltage = Volts.of(6.0)
        val retractionVoltage = Volts.of(-6.0)
        val currentThreshold: Measure<Current> = Amps.of(25.0)
    }
}