package frc.robot.subsystems.climb

import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import lib.math.units.into

class ClimbIOSim : ClimbIO {
    val system: LinearSystem<N2, N1, N2> = LinearSystemId.createDCMotorSystem(
        25.58,
        0.04
    )
    val leftArmMotor: DCMotorSim = DCMotorSim(
        system,
        DCMotor.getNEO(1),
        16.0
    )
    val rightArmMotor: DCMotorSim = DCMotorSim(
        system,
        DCMotor.getNEO(1),
        16.0
    )

    val leftArmVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
    val rightArmVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)

    override fun updateInputs(inputs: ClimbIO.ClimbIOInputs) {
        leftArmMotor.update(0.02)
        rightArmMotor.update(0.02)

        inputs.leftArmVelocity.mut_replace(
            (leftArmMotor.angularVelocityRPM / 60.0) * (Math.PI * 0.750),
            Units.InchesPerSecond
        )
        inputs.rightArmVelocity.mut_replace(
            (rightArmMotor.angularVelocityRPM / 60.0) * (Math.PI * 0.750),
            Units.InchesPerSecond
        )

        inputs.leftArmPosition.mut_replace(
            leftArmMotor.angularPositionRotations * (Math.PI * 0.750),
            Units.Inches
        )

        inputs.rightArmPosition.mut_replace(
            rightArmMotor.angularPositionRotations * (Math.PI * 0.750),
            Units.Inches
        )

        inputs.leftArmAppliedVoltage.mut_replace(leftArmVoltage)
        inputs.rightArmAppliedVoltage.mut_replace(rightArmVoltage)
    }

    override fun setLeftArmPower(power: Double) {
        leftArmVoltage.mut_replace(power * 12.0, Units.Volts)
        leftArmMotor.setInputVoltage(leftArmVoltage.into(Units.Volts))
    }

    override fun setRightArmPower(power: Double) {
        rightArmVoltage.mut_replace(power * 12.0, Units.Volts)
        rightArmMotor.setInputVoltage(rightArmVoltage.into(Units.Volts))
    }

    override fun stop() {
        leftArmVoltage.mut_replace(0.0, Units.Volts)
        rightArmVoltage.mut_replace(0.0, Units.Volts)
        leftArmMotor.setInputVoltage(leftArmVoltage.into(Units.Volts))
        rightArmMotor.setInputVoltage(rightArmVoltage.into(Units.Volts))
    }

    override fun resetLeftArmPosition() {
        leftArmMotor.setState(0.0, 0.0)
    }

    override fun resetRightArmPosition() {
        rightArmMotor.setState(0.0, 0.0)
    }
}