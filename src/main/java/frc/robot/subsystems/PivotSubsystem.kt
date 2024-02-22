package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants
import frc.robot.Robot
import lib.math.units.Rotation
import lib.math.units.into
import lib.putMap

object PivotSubsystem : SubsystemBase() {
    private val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(Constants.PivotConstants.ABSOLUTE_ENCODER_PORT)
    private val pivotMotor: CANSparkMax = CANSparkMax(Constants.PivotConstants.PIVOT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless)
    private val relativeEncoder: RelativeEncoder = pivotMotor.encoder   
    val pidController: SparkPIDController = pivotMotor.pidController

    private var encoderSet: Boolean = false

    private val interpolatedAimTable: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()

    init{
        configureMotor()
        pivotMotor.stopMotor()
        interpolatedAimTable.putMap(Constants.PivotConstants.distanceMap)
    }

    private fun configureMotor(){
        pivotMotor.restoreFactoryDefaults()
        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        pivotMotor.burnFlash()
    }

    fun configureEncoders() {
        relativeEncoder.positionConversionFactor = 1 / Constants.PivotConstants.REL_ENCODER_CONVERSION
        println("Absolute Encoder pos at config time: ${getAbsEncoder()}")
        relativeEncoder.setPosition(getAbsEncoder())
    }

    fun configurePID(){
        pidController.p = 10.0
        pidController.i = 0.005
        pidController.d = 50.0
        pidController.setFeedbackDevice(relativeEncoder)
    }

    fun getAbsEncoder(): Double {
        return (absoluteEncoder.absolutePosition / Constants.PivotConstants.ABS_ENCODER_CONVERSION) - Constants.PivotConstants.ABSOLUTE_OFFSET
    }

    fun rawMotorSpeed(speed: Double){
        pivotMotor.set(speed)
    }

    fun setTargetPosition(position: Double){
        pidController.setReference(position, CANSparkBase.ControlType.kPosition)
    }

    fun setTargetPosition(position: Rotation){
        pidController.setReference(position into Units.Rotations, CANSparkBase.ControlType.kPosition)
    }

    fun getPosition() = relativeEncoder.position

    override fun periodic() {
        if(Robot.isEnabled && !encoderSet){
            configureEncoders()
            configurePID()
            encoderSet = true
        }
    }



}