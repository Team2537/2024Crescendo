package frc.robot.subsystems.launcher

import com.revrobotics.SparkPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.math.util.Units.lbsToKilograms
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.simulation.DIOSim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.robot.Constants
import kotlin.math.pow

class LauncherIOSim : LauncherIO {
    val wheelMass = lbsToKilograms(0.12)
    val wheelRadius = inchesToMeters(1.5)
    val flywheelMOI: Double = 4 * (0.5 * (wheelMass * wheelRadius).pow(2))

    val topVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
    val bottomVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)

    val sensorSim: DIOSim = DIOSim(Constants.LauncherConstants.RIGHT_NOTE_DETECTOR)

    val topFlywheels: FlywheelSim = FlywheelSim(
        DCMotor.getNeoVortex(1),
        1.0,
        flywheelMOI
    )

    val bottomFlywheels: FlywheelSim = FlywheelSim(
        DCMotor.getNeoVortex(1),
        1.0,
        flywheelMOI
    )

    override fun updateInputs(inputs: LauncherIO.LauncherIOInputs) {
        inputs.topFlywheelsVelocity.mut_replace(
            topFlywheels.angularVelocityRPM * (2 * Math.PI * wheelRadius),
            Units.MetersPerSecond
        )
        inputs.bottomFlywheelsVelocity.mut_replace(
            bottomFlywheels.angularVelocityRPM * (2 * Math.PI * wheelRadius),
            Units.MetersPerSecond
        )

        inputs.topFlywheelAppliedVoltage.mut_replace(topVoltage)
        inputs.bottomFlywheelAppliedVoltage.mut_replace(bottomVoltage)

        inputs.rightNoteDetected = sensorSim.value

        inputs.rollerVelocity = MutableMeasure.zero(Units.MetersPerSecond)
        inputs.rollerPosition = MutableMeasure.zero(Units.Meters)
    }

    override fun setRollerPower(power: Double) {
        TODO("Not yet implemented")
    }

    override fun setFlywheelPower(power: Double) {
        TODO("Not yet implemented")
    }

    override fun runSetpoint(
        velocity: Measure<Velocity<Distance>>,
        arbFF_t: Double,
        arbFF_b: Double,
        arbFFUnits: SparkPIDController.ArbFFUnits
    ) {
        TODO("Not yet implemented")
    }

    override fun runRollerSetpoint(position: Measure<Distance>) {
        TODO("Not yet implemented")
    }

    override fun setFlywheelsBrakeMode(brake: Boolean) {
        TODO("Not yet implemented")
    }

    override fun setRollerBrakeMode(brake: Boolean) {
        TODO("Not yet implemented")
    }

    override fun stopFlywheels() {
        TODO("Not yet implemented")
    }

    override fun stopRoller() {
        TODO("Not yet implemented")
    }

    override fun setFlywheelVoltage(voltage: Measure<Voltage>) {
        TODO("Not yet implemented")
    }

    override fun setRollerVoltage(voltage: Measure<Voltage>) {
        TODO("Not yet implemented")
    }

}