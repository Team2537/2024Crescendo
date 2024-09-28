package frc.robot.subsystems.intake

import edu.wpi.first.hal.SimBoolean
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import lib.math.units.into

class IntakeIOSim : IntakeIO {
    private val intakeSim: FlywheelSim = FlywheelSim(
        DCMotor.getNeo550(1),
        2.0,
        0.0000719
    )

    private val intakeSensorBool: SimBoolean = SimBoolean(1)
    private val exitSensorBool: SimBoolean = SimBoolean(2)

    private var cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.intakeSensor = intakeSensorBool.get()
        inputs.exitSensor = exitSensorBool.get()
        inputs.intakeLinearVelocity = intakeSim.angularVelocityRadPerSec * (Units.inchesToMeters(0.84) / 2)
        inputs.intakeSupplyVoltage = 12.0
        inputs.intakeMotorVoltage = cachedVoltage into Volts
        inputs.intakeStatorCurrent = intakeSim.currentDrawAmps
    }

    override fun applyVoltage(voltage: Measure<Voltage>) {
        cachedVoltage.mut_replace(voltage)
        intakeSim.setInputVoltage(voltage into Volts)
    }

    override fun stop() {
        applyVoltage(Volts.of(0.0))
    }
}