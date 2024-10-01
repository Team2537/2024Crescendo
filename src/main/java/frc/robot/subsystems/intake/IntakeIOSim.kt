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
import lib.math.units.into

class IntakeIOSim(gearing: Double, moi: Double, private val rollerDiameter: Measure<Distance>) : IntakeIO {
    private val intakeSim: FlywheelSim = FlywheelSim(
        DCMotor.getNeo550(1),
        gearing,
        moi
    )

    private val intakeSensorBool: SimBoolean = SimBoolean(1)
    private val exitSensorBool: SimBoolean = SimBoolean(2)

    private var cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.intakeSensorTriggered = intakeSensorBool.get()
        inputs.exitSensorTriggered = exitSensorBool.get()
        intakeSim.update(0.02)
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