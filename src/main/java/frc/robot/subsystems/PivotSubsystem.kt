package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants
import lib.math.units.*

class PivotSubsystem : SubsystemBase() {
    private val pivotMotor: CANSparkMax = CANSparkMax(
        PivotConstants.PIVOT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless
    )

    private val pivotEncoder: RelativeEncoder = pivotMotor.encoder

    private val pivotPIDController: SparkPIDController = pivotMotor.pidController

    private val homingSensor: DigitalInput = DigitalInput(PivotConstants.HOMING_SENSOR_PORT)

    private val layout: ShuffleboardLayout = Shuffleboard.getTab("Subsystems")
        .getLayout("Pivot", BuiltInLayouts.kList)

    private val angle: MutableMeasure<Angle> = MutableMeasure.zero(Units.Radians)
    private var setpoint: Double = 0.0 // Radians, no units because its an underlying value so whatever

    /**
     * The feedforward controller for the pivot
     * Uses radians per second as the velocity unit
     */
    private val feedforward = ArmFeedforward(
        PivotConstants.kS,
        PivotConstants.kG,
        PivotConstants.kV,
        PivotConstants.kA,
    )

    private val constraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(1.0, 0.5)
    private lateinit var profile: TrapezoidProfile
    private val timer: Timer = Timer()

    private lateinit var startState: TrapezoidProfile.State
    private lateinit var endState: TrapezoidProfile.State

    private lateinit var targetState: TrapezoidProfile.State

    init {
        pivotMotor.restoreFactoryDefaults()

        pivotEncoder.positionConversionFactor = PivotConstants.REL_ENCODER_CONVERSION
        pivotEncoder.velocityConversionFactor = PivotConstants.REL_ENCODER_CONVERSION

        pivotEncoder.position = 0.0

        pivotPIDController.p = PivotConstants.kP
        pivotPIDController.i = PivotConstants.kI
        pivotPIDController.d = PivotConstants.kD
        pivotPIDController.setIMaxAccum(100.0, 0) // We need to figure out what this is

        pivotMotor.setSmartCurrentLimit(40)

        layout.addDouble("Encoder Position") { pivotEncoder.position }
        layout.addDouble("Encoder Velocity") { pivotEncoder.velocity }
        layout.addBoolean("Homing Sensor") { homingSensor.get() }
        layout.addDouble("Output") { pivotMotor.appliedOutput * pivotMotor.busVoltage }
        layout.addDouble("IAccum") { pivotPIDController.iAccum }

        timer.start()
        updateProfile()
    }


    val homed: Boolean
        get() = homingSensor.get()

    fun updateProfile() {
        startState = TrapezoidProfile.State(pivotEncoder.position, pivotEncoder.velocity)
        endState = TrapezoidProfile.State(setpoint, 0.0)
        profile = TrapezoidProfile(constraints)
        timer.reset()
    }

    fun getAngle(): MutableMeasure<Angle> = angle.mut_replace(pivotEncoder.position.inRadians)


    private fun setVoltage(voltage: Measure<Voltage>) {
        pivotMotor.setVoltage(voltage.into(Units.Volts))
    }

    fun setTargetPosition(position: Measure<Angle>) {
        if(setpoint != position.into(Units.Radians)) {
            setpoint = position.into(Units.Radians)
            updateProfile()
        }
    }

    fun runAutomatic(){
        val elapsedTime = timer.get()
        targetState = if(profile.isFinished(elapsedTime)){
            endState
        } else {
            profile.calculate(elapsedTime, startState, endState)
        }

        val feedforwardVoltage = feedforward.calculate(pivotEncoder.position, targetState.velocity)

        pivotPIDController.setReference(
            targetState.position,
            CANSparkBase.ControlType.kPosition,
            0,
            feedforwardVoltage,
            SparkPIDController.ArbFFUnits.kVoltage
        )
    }


//    fun calculateFeedforward(
//        angle: Measure<Angle>,
//        velocity: Measure<Velocity<Angle>>
//    ): Measure<Voltage> {
//        return feedforward.calculate(angle.into(Units.Radians), velocity.into(Units.RadiansPerSecond)).inVolts
//    }
}