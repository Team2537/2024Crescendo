package frc.robot.subsystems.pivot

import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.simulation.DIOSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.Constants
import lib.math.units.into
import edu.wpi.first.math.util.Units as Conversions

class PivotIOSim : PivotIO {

    val homingSensor: DIOSim = DIOSim(Constants.PivotConstants.HOMING_SENSOR_PORT)
    val armSystem: LinearSystem<N2, N1, N1>? = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(1),
        0.05245676,
        Constants.PivotConstants.GEARBOX_RATIO * Constants.PivotConstants.PULLEY_RATIO
    )
    val armSim: SingleJointedArmSim = SingleJointedArmSim(
        armSystem,
        DCMotor.getNEO(1),
        Constants.PivotConstants.GEARBOX_RATIO * Constants.PivotConstants.PULLEY_RATIO,
        Conversions.inchesToMeters(20.0),
        Conversions.degreesToRadians(0.0),
        Conversions.degreesToRadians(90.0),
        true,
        0.0
    )

    val appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)

    init {}

    override fun updateInputs(inputs: PivotIO.PivotIOInputs) {
        armSim.update(0.02)

        inputs.absoluteAngle.mut_replace(armSim.angleRads, Units.Radians)
        inputs.relativeAngle.mut_replace(armSim.angleRads, Units.Radians)
        inputs.homingSensorTriggered = homingSensor.value
        inputs.appliedVoltage.mut_replace(appliedVoltage)
    }

    override fun setRawVoltage(voltage: Measure<Voltage>) {
        appliedVoltage.mut_replace(voltage)
        armSim.setInputVoltage(voltage.into(Units.Volts))
    }

    override fun syncEncoders() {
        println("Syncing encoders")
    }

    override fun zeroRelativeEncoder(position: Measure<Angle>) {
        armSim.setState(position.into(Units.Radians), 0.0)
    }

    override fun runSetpoint(setpoint: Measure<Angle>, arbFFUnits: Measure<Voltage>) {
        TODO("Not yet implemented")
    }

    override fun stop() {
        appliedVoltage.mut_replace(0.0, Units.Volt)
        armSim.setInputVoltage(0.0)
    }

    override fun getRawAbs(): Double {
        return 0.0
    }
}