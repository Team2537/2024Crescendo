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
import edu.wpi.first.wpilibj2.command.Command
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
    private val velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RadiansPerSecond)

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

    val constraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(1.0, 0.5)


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

    }


    val homed: Boolean
        get() = homingSensor.get()


    fun getAngle(): MutableMeasure<Angle> = angle.mut_replace(pivotEncoder.position.inRadians)
    fun getVelocity(): MutableMeasure<Velocity<Angle>> = velocity.mut_replace(pivotEncoder.velocity.inRPS)


    private fun setVoltage(voltage: Measure<Voltage>) {
        pivotMotor.setVoltage(voltage.into(Units.Volts))
    }


    fun calculateFeedforward(
        angle: Measure<Angle>,
        velocity: Measure<Velocity<Angle>>
    ): Measure<Voltage> {
        return feedforward.calculate(angle.into(Units.Radians), velocity.into(Units.RadiansPerSecond)).inVolts
    }

    fun applyPID(angle: Measure<Angle>, feedforward: Measure<Voltage>){
        pivotPIDController.setReference(
            angle.into(Units.Radians),
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.into(Units.Volts),
            SparkPIDController.ArbFFUnits.kVoltage
        )
    }

    fun stop(){
        pivotMotor.stopMotor()
    }

    fun homeCommand(): Command {
        return this.runEnd(
            { pivotMotor.set(-0.2) },
            { stop() }
        ).until { homed }
    }
}