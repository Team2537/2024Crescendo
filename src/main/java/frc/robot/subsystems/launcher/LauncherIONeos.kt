package frc.robot.subsystems.launcher

import com.revrobotics.*
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants
import lib.math.units.into

class LauncherIONeos : LauncherIO {
    /** Motor for the top flywheels */
    val topFlywheels: CANSparkFlex = CANSparkFlex(
        Constants.LauncherConstants.TOP_FLYWHEELS,
        CANSparkLowLevel.MotorType.kBrushless
    )

    /** Motor for the bottom flywheels */
    val bottomFlywheels: CANSparkFlex = CANSparkFlex(
        Constants.LauncherConstants.BOTTOM_FLYWHEELS,
        CANSparkLowLevel.MotorType.kBrushless
    )

    /** Motor for the roller that feeds the launcher */
    val rollerMotor: CANSparkMax = CANSparkMax(
        Constants.LauncherConstants.ROLLER_MOTOR,
        CANSparkLowLevel.MotorType.kBrushless
    )

//    val leftNoteDetector: DigitalInput = DigitalInput(Constants.LauncherConstants.LEFT_NOTE_DETECTOR)
    /** Sensor placed on the right side of the launcher to detect the note */
    val rightNoteDetector: DigitalInput = DigitalInput(Constants.LauncherConstants.RIGHT_NOTE_DETECTOR)

    init {
        // Reset the motors to factory defaults to ensure that they are in a known state
        topFlywheels.restoreFactoryDefaults()
        bottomFlywheels.restoreFactoryDefaults()
        rollerMotor.restoreFactoryDefaults()

        // Invert the bottom flywheels so that both sets of flywheels are pushing or pulling together
        bottomFlywheels.inverted = true

        // Set the current limits so that the motors don't burn out
        topFlywheels.setSmartCurrentLimit(40)
        bottomFlywheels.setSmartCurrentLimit(40)
        rollerMotor.setSmartCurrentLimit(40)

        // Set the conversion factors for the encoders so that they are in rotations, with no gearing
        topFlywheels.encoder.positionConversionFactor = 1.0 / (3 * Math.PI)
        bottomFlywheels.encoder.positionConversionFactor = 1.0 / (3 * Math.PI)
        topFlywheels.encoder.velocityConversionFactor = 1.0 / (3 * Math.PI)
        bottomFlywheels.encoder.velocityConversionFactor = 1.0 / (3 * Math.PI)

        // Set the motors to brake mode so that they don't move (as easily) when disabled
        topFlywheels.setIdleMode(CANSparkBase.IdleMode.kBrake)
        bottomFlywheels.setIdleMode(CANSparkBase.IdleMode.kBrake)

        // Set the PID values for the roller
        rollerMotor.pidController.p = 0.25
        rollerMotor.pidController.i = 0.0
        rollerMotor.pidController.d = 0.0

        rollerMotor.burnFlash()
        topFlywheels.burnFlash()
        bottomFlywheels.burnFlash()

        rollerMotor.encoder.setPosition(0.0)
        topFlywheels.encoder.setPosition(0.0)
        bottomFlywheels.encoder.setPosition(0.0)
    }

    override fun updateInputs(inputs: LauncherIO.LauncherIOInputs) {
        inputs.rollerVelocity.mut_replace(rollerMotor.encoder.velocity, Units.MetersPerSecond)
        inputs.rollerPosition.mut_replace(rollerMotor.encoder.position, Units.Meters)
        inputs.topFlywheelsVelocity.mut_replace(topFlywheels.encoder.velocity, Units.MetersPerSecond)
        inputs.bottomFlywheelsVelocity.mut_replace(bottomFlywheels.encoder.velocity, Units.MetersPerSecond)
        inputs.topFlywheelAppliedVoltage.mut_replace(
            topFlywheels.appliedOutput * topFlywheels.busVoltage, Units.Volts
        )
        inputs.bottomFlywheelAppliedVoltage.mut_replace(
            bottomFlywheels.appliedOutput * bottomFlywheels.busVoltage, Units.Volts
        )
        inputs.rightNoteDetected = rightNoteDetector.get()
    }

    override fun setRollerPower(power: Double) {
        rollerMotor.set(power)
    }

    override fun setFlywheelPower(power: Double) {
        topFlywheels.set(power)
        bottomFlywheels.set(power)
    }

    override fun runSetpoint(
        velocity: Measure<Velocity<Distance>>,
        arbFF_t: Double,
        arbFF_b: Double,
        arbFFUnits: SparkPIDController.ArbFFUnits
    ) {
        topFlywheels.pidController.setReference(
            velocity.baseUnitMagnitude(),
            CANSparkBase.ControlType.kVelocity,
            0,
            arbFF_t,
            arbFFUnits
        )

        bottomFlywheels.pidController.setReference(
            velocity.baseUnitMagnitude(),
            CANSparkBase.ControlType.kVelocity,
            0,
            arbFF_b,
            arbFFUnits
        )
    }

    override fun runRollerSetpoint(position: Measure<Distance>) {
        rollerMotor.pidController.setReference(
            position.baseUnitMagnitude(),
            CANSparkBase.ControlType.kPosition
        )
    }

    override fun setFlywheelsBrakeMode(brake: Boolean) {
        if (brake) {
            topFlywheels.setIdleMode(CANSparkBase.IdleMode.kBrake)
            bottomFlywheels.setIdleMode(CANSparkBase.IdleMode.kBrake)
        } else {
            topFlywheels.setIdleMode(CANSparkBase.IdleMode.kCoast)
            bottomFlywheels.setIdleMode(CANSparkBase.IdleMode.kCoast)
        }
    }

    override fun setRollerBrakeMode(brake: Boolean) {
        if (brake) {
            rollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        } else {
            rollerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        }
    }

    override fun stopFlywheels() {
        topFlywheels.stopMotor()
        bottomFlywheels.stopMotor()
    }

    override fun stopRoller() {
        rollerMotor.stopMotor()
    }

    override fun setFlywheelVoltage(topVoltage: Measure<Voltage>, bottomVoltage: Measure<Voltage>) {
        topFlywheels.setVoltage(topVoltage.into(Units.Volts))
        bottomFlywheels.setVoltage(bottomVoltage.into(Units.Volts))
    }

    override fun setRollerVoltage(voltage: Measure<Voltage>) {
        rollerMotor.setVoltage(voltage.into(Units.Volts))
    }
}