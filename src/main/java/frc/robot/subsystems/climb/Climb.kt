package frc.robot.subsystems.climb

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import kotlin.math.sin

class Climb : SubsystemBase() {
    private val leftArmIO: ClimberArmIO = when(Constants.RobotConstants.mode){
        Constants.RobotConstants.Mode.REAL -> ClimberArmNeo(
            leftID,
            false,
            gearing,
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
            true,
            gearing,
        )
        Constants.RobotConstants.Mode.SIM -> ClimberArmIOSim(
            DCMotor.getNEO(1),
            gearing, load, drumRadius, maxExtension
        )
        Constants.RobotConstants.Mode.REPLAY -> object : ClimberArmIO {}
    }

    val leftInputs: ClimberArmIO.ClimberArmInputs = ClimberArmIO.ClimberArmInputs()
    val rightInputs: ClimberArmIO.ClimberArmInputs = ClimberArmIO.ClimberArmInputs()

    override fun periodic() {
        leftArmIO.updateInputs(leftInputs)
        rightArmIO.updateInputs(rightInputs)

        Logger.processInputs("climb/leftArm", leftInputs)
        Logger.processInputs("climb/rightArm", rightInputs)

        Logger.recordOutput("climb/leftArmPose", Pose3d.struct, Pose3d(
            Translation3d(
                0.0,
                Units.inchesToMeters(10.0),
                (leftInputs.relativePosition into Radians) * (drumRadius into Meters) + Units.inchesToMeters(8.0)
            ),
            Rotation3d()
        ))

        Logger.recordOutput("climb/rightArmPose", Pose3d.struct, Pose3d(
            Translation3d(
                0.0,
                Units.inchesToMeters(-10.0),
                (rightInputs.relativePosition into Radians) * (drumRadius into Meters) + Units.inchesToMeters(8.0)
            ),
            Rotation3d()
        ))
    }

    fun getSineCommand() = run {
        val voltage = sin(Timer.getFPGATimestamp()) * 12.0
        leftArmIO.setVoltage(Volts.of(voltage), false)
        rightArmIO.setVoltage(Volts.of(voltage), false)
    }

    companion object {
        val leftID = 18
        val rightID = 15

        val maxExtension = Inches.of(60.0)
        val drumRadius = Inches.of(0.750)

        val load = Constants.RobotConstants.robotWeight.divide(2.0)
        val gearing = 16/1.0
    }
}