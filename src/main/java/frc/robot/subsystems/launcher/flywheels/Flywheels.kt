package frc.robot.subsystems.launcher.flywheels

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.RobotConstants.mode
import lib.ControllerGains
import org.littletonrobotics.junction.Logger

class Flywheels : SubsystemBase("flywheels") {
    val topFlywheelIO: FlywheelIO = when(mode) {
        Constants.RobotConstants.Mode.REAL -> FlywheelIONeo(
            topID,
            false,
            ControllerGains()
        )
        Constants.RobotConstants.Mode.SIM -> FlywheelIOSim(
            DCMotor.getNeoVortex(1),
            moiKgM2 = MOI,
            ControllerGains(
                kV = 12.0/6900.0,
                kP = 0.17
            ))
        Constants.RobotConstants.Mode.REPLAY -> object : FlywheelIO {}
    }
    val bottomFlywheelIO: FlywheelIO = when(mode) {
        Constants.RobotConstants.Mode.REAL -> FlywheelIONeo(
            bottomID,
            true,
            ControllerGains()
        )
        Constants.RobotConstants.Mode.SIM -> FlywheelIOSim(
            DCMotor.getNeoVortex(1),
            moiKgM2 = MOI,
            ControllerGains(
                kV = 12.0/6900.0,
                kP = 0.17
            ))
        Constants.RobotConstants.Mode.REPLAY -> object : FlywheelIO {}
    }

    val topInputs: FlywheelIO.FlywheelInputs = FlywheelIO.FlywheelInputs()
    val bottomInputs: FlywheelIO.FlywheelInputs = FlywheelIO.FlywheelInputs()

    private val goalVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RPM)

    fun getPrimeCommand(targetVelocity: Measure<Velocity<Angle>>) = runOnce {
        goalVelocity.mut_replace(targetVelocity)
        topFlywheelIO.setVelocity(targetVelocity)
        bottomFlywheelIO.setVelocity(targetVelocity)
    }

    fun getWaitUntilReadyCommand() = Commands.waitUntil {
        topInputs.velocity.isNear(goalVelocity, 0.05) &&
        bottomInputs.velocity.isNear(goalVelocity, 0.05)
    }

    override fun periodic() {
        topFlywheelIO.updateInputs(topInputs)
        bottomFlywheelIO.updateInputs(bottomInputs)

        Logger.processInputs("launcher/top", topInputs)
        Logger.processInputs("launcher/bottom", bottomInputs)
    }

    companion object {
        val topID: Int = 200 // TODO: Update this value
        val bottomID: Int = 190 // TODO: Update this value

        val MOI: Double = 0.001317 * 4
    }
}