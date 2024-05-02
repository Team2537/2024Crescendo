package frc.robot.subsystems.pivot

import com.revrobotics.*
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.robot.Constants
import lib.math.units.degrees
import lib.math.units.into

class PivotIONeos : PivotIO {
    val pivotMotor: CANSparkMax = CANSparkMax(
        Constants.PivotConstants.PIVOT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless
    )

    val relativeEncoder: RelativeEncoder = pivotMotor.encoder

    val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(Constants.PivotConstants.ABSOLUTE_ENCODER_PORT)

    val homingSensor: DigitalInput = DigitalInput(Constants.PivotConstants.HOMING_SENSOR_PORT)

    val pivotPID: SparkPIDController = pivotMotor.pidController

    val setpoint: MutableMeasure<Angle> = MutableMeasure.zero(Units.Radians)

    init {
        pivotMotor.restoreFactoryDefaults()
        relativeEncoder.positionConversionFactor = Constants.PivotConstants.REL_ENCODER_CONVERSION
        absoluteEncoder.distancePerRotation = Constants.PivotConstants.ABS_ENCODER_CONVERSION
        absoluteEncoder.positionOffset = Constants.PivotConstants.ABSOLUTE_OFFSET

        pivotPID.p = Constants.PivotConstants.kP
        pivotPID.i = Constants.PivotConstants.kI
        pivotPID.d = Constants.PivotConstants.kD
        pivotPID.ff = 0.0

        pivotMotor.setSmartCurrentLimit(40)

        pivotMotor.inverted = true

        syncEncoders()

        pivotMotor.burnFlash()
    }

    override fun updateInputs(inputs: PivotIO.PivotIOInputs) {
        inputs.absoluteAngle.mut_replace(absoluteEncoder.distance.degrees)
        inputs.relativeAngle.mut_replace(relativeEncoder.position.degrees)
        inputs.homingSensorTriggered = homingSensor.get()
        inputs.appliedVoltage.mut_replace(pivotMotor.appliedOutput * pivotMotor.busVoltage, Units.Volts)
        inputs.setpoint.mut_replace(setpoint)
    }

    override fun setRawVoltage(voltage: Measure<Voltage>) {
        pivotMotor.setVoltage(voltage.baseUnitMagnitude())
    }

    override fun syncEncoders() {
        relativeEncoder.position = absoluteEncoder.distance
    }

    override fun zeroRelativeEncoder(position: Measure<Angle>) {
        relativeEncoder.position = position.into(Units.Degrees)
    }

    override fun runSetpoint(setpoint: Measure<Angle>, arbFFUnits: Measure<Voltage>) {
        this.setpoint.mut_replace(setpoint)
        pivotPID.setReference(
            setpoint.into(Units.Degrees),
            CANSparkBase.ControlType.kPosition,
            0,
            arbFFUnits.into(Units.Volts),
            SparkPIDController.ArbFFUnits.kVoltage
        )
    }

    override fun stop() {
        pivotMotor.stopMotor()
    }

}