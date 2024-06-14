package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import lib.math.units.inMPS
import lib.math.units.inMeters
import lib.math.units.into

class LauncherSubsystem : SubsystemBase() {
    private val topFlywheels: CANSparkFlex =
        CANSparkFlex(Constants.LauncherConstants.TOP_FLYWHEELS, CANSparkLowLevel.MotorType.kBrushless)
    private val bottomFlywheels: CANSparkFlex =
        CANSparkFlex(Constants.LauncherConstants.BOTTOM_FLYWHEELS, CANSparkLowLevel.MotorType.kBrushless)

    private val roller: CANSparkMax =
        CANSparkMax(Constants.LauncherConstants.ROLLER_MOTOR, CANSparkLowLevel.MotorType.kBrushless)

    private val noteSensor: DigitalInput = DigitalInput(Constants.LauncherConstants.NOTE_DETECTOR)

    private val topFlywheelPIDController: SparkPIDController = topFlywheels.pidController
    private val bottomFlywheelPIDController: SparkPIDController = bottomFlywheels.pidController
    private val rollerPIDController: SparkPIDController = roller.pidController

    private val topFlywheelFeedforward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(-0.20832, 0.11109, 0.024896)
    private val bottomFlywheelFeedforward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(0.035079, 0.10631, 0.0080339)

    private val layout = Shuffleboard.getTab("Subsystems").getLayout("Launcher", BuiltInLayouts.kList)

    private val topFlywheelVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)
    private val bottomFlywheelVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)
    private val rollerPosition: MutableMeasure<Distance> = MutableMeasure.zero(Units.Meters)

    init {
        topFlywheels.restoreFactoryDefaults()
        bottomFlywheels.restoreFactoryDefaults()
        roller.restoreFactoryDefaults()

        bottomFlywheels.inverted = true

        topFlywheels.setSmartCurrentLimit(40)
        bottomFlywheels.setSmartCurrentLimit(40)
        roller.setSmartCurrentLimit(40)

        topFlywheels.encoder.positionConversionFactor = Constants.LauncherConstants.FLYWHEEL_CIRCUMFERENCE
        bottomFlywheels.encoder.positionConversionFactor = Constants.LauncherConstants.FLYWHEEL_CIRCUMFERENCE

        topFlywheels.encoder.velocityConversionFactor = Constants.LauncherConstants.FLYWHEEL_CIRCUMFERENCE / 60.0
        bottomFlywheels.encoder.velocityConversionFactor = Constants.LauncherConstants.FLYWHEEL_CIRCUMFERENCE / 60.0

        roller.encoder.positionConversionFactor = Constants.LauncherConstants.ROLLER_CIRCUMFERENCE
        roller.encoder.velocityConversionFactor = Constants.LauncherConstants.ROLLER_CIRCUMFERENCE / 60.0

        topFlywheels.idleMode = CANSparkBase.IdleMode.kBrake
        bottomFlywheels.idleMode = CANSparkBase.IdleMode.kBrake
        roller.idleMode = CANSparkBase.IdleMode.kBrake

        rollerPIDController.p = 0.25
        rollerPIDController.i = 0.0
        rollerPIDController.d = 0.0

        topFlywheelPIDController.p = 0.0001
        topFlywheelPIDController.i = 0.0
        topFlywheelPIDController.d = 0.0

        bottomFlywheelPIDController.p = 0.0001
        bottomFlywheelPIDController.i = 0.0
        bottomFlywheelPIDController.d = 0.0

        topFlywheels.burnFlash()
        bottomFlywheels.burnFlash()
        roller.burnFlash()

        layout.addDouble("Top Flywheel Velocity") { topFlywheels.encoder.velocity }
        layout.addDouble("Bottom Flywheel Velocity") { bottomFlywheels.encoder.velocity }
        layout.addDouble("Roller Position") { roller.encoder.position }
        layout.addBoolean("Note Sensor") { !noteSensor.get() }
    }

    val noteDetected: Boolean
        get() = !noteSensor.get()

    fun getTopFlywheelVelocity(): MutableMeasure<Velocity<Distance>> =
        topFlywheelVelocity.mut_replace(topFlywheels.encoder.velocity.inMPS)

    fun getBottomFlywheelVelocity(): MutableMeasure<Velocity<Distance>> =
        bottomFlywheelVelocity.mut_replace(bottomFlywheels.encoder.velocity.inMPS)

    fun getRollerPosition(): MutableMeasure<Distance> =
        rollerPosition.mut_replace(roller.encoder.position.inMeters)

    fun setFlywheelVelocity(velocity: Measure<Velocity<Distance>>){
        topFlywheelPIDController.setReference(
            velocity.into(Units.MetersPerSecond),
            CANSparkBase.ControlType.kVelocity,
            0,
            topFlywheelFeedforward.calculate(velocity.into(Units.MetersPerSecond)),
            SparkPIDController.ArbFFUnits.kVoltage
        )

        bottomFlywheelPIDController.setReference(
            velocity.into(Units.MetersPerSecond),
            CANSparkBase.ControlType.kVelocity,
            0,
            bottomFlywheelFeedforward.calculate(velocity.into(Units.MetersPerSecond)),
            SparkPIDController.ArbFFUnits.kVoltage
        )
    }

    fun setRollerPosition(position: Measure<Distance>){
        rollerPIDController.setReference(
            position.into(Units.Meters),
            CANSparkBase.ControlType.kPosition,
        )
    }

    fun stopRoller(){
        roller.stopMotor()
    }

    fun stopFlywheels(){
        topFlywheels.stopMotor()
        bottomFlywheels.stopMotor()
    }

    fun setFlywheelBrakes(brake: Boolean){
        topFlywheels.idleMode = if(brake) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast
        bottomFlywheels.idleMode = if(brake) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast
    }

    fun setRollerBrake(brake: Boolean){
        roller.idleMode = if(brake) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast
    }

    fun setRawFlywheelVoltage(voltage: Double){
        topFlywheels.setVoltage(voltage)
        bottomFlywheels.setVoltage(voltage)
    }

    fun setRawRollerVoltage(voltage: Double){
        roller.setVoltage(voltage)
    }
}