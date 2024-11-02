package frc.robot.subsystems.climb

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import lib.math.units.into

class ClimberArmIOSim(
    private val motor: DCMotor,
    private val gearing: Double,
    private val load: Measure<Mass>,
    private val drumRadius: Measure<Distance>,
    private val maxExtension: Measure<Distance>
) : ClimberArmIO {
    private val elevatorSim = ElevatorSim(
        motor,
        gearing,
        load into Kilograms,
        drumRadius into Meters,
        0.0, // Minimum extension
        maxExtension into Meters,
        false, // Simulate gravity,
        0.0 // Initial position
    )

    private val cachedVoltage = MutableMeasure.zero(Volts)

    override fun updateInputs(inputs: ClimberArmIO.ClimberArmInputs) {
        elevatorSim.update(0.02)

        inputs.velocity.mut_replace(elevatorSim.velocityMetersPerSecond, MetersPerSecond)
        inputs.relativePosition.mut_replace(elevatorSim.positionMeters, Meters)
        inputs.appliedCurrent.mut_replace(Amps.of(elevatorSim.currentDrawAmps))
        inputs.appliedVoltage.mut_replace(cachedVoltage)
    }

    override fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean) {
        cachedVoltage.mut_replace(voltage)
        elevatorSim.setInputVoltage(voltage into Volts)
    }

    override fun stop() {
        elevatorSim.setInputVoltage(0.0)
    }
}