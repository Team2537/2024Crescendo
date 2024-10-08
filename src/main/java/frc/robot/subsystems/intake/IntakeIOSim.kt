package frc.robot.subsystems.intake

import edu.wpi.first.hal.SimBoolean
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meter
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import lib.math.units.into
import java.util.function.BooleanSupplier

class IntakeIOSim(
    gearing: Double,
    moi: Double,
    private val rollerDiameter: Measure<Distance>,
    private val intakeSensorBool: BooleanSupplier,
    private val exitSensorBool: BooleanSupplier
) : IntakeIO {
    private val intakeSim: FlywheelSim = FlywheelSim(
        DCMotor.getNeo550(1),
        gearing,
        moi
    )

    private var cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        intakeSim.update(0.02)

        inputs.intakeSensorTriggered = intakeSensorBool.asBoolean
        inputs.exitSensorTriggered = exitSensorBool.asBoolean
        inputs.linearVelocity
            .mut_replace(intakeSim.angularVelocityRadPerSec * ((rollerDiameter into Meter) / 2), MetersPerSecond) // Convert from rad/s to m/s
        inputs.supplyVoltage.mut_replace(RobotController.getBatteryVoltage(), Volts)
        inputs.motorVoltage.mut_replace(cachedVoltage)
        inputs.statorCurrent.mut_replace(intakeSim.currentDrawAmps, Amps)
    }

    override fun setVoltage(voltage: Measure<Voltage>) {
        cachedVoltage.mut_replace(voltage)
        intakeSim.setInputVoltage(voltage into Volts)
    }

    override fun stop() {
        setVoltage(Volts.of(0.0))
    }
}