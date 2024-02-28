package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
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

    private val tab = Shuffleboard.getTab("Pivot")
    init{
        configureMotor()
        pivotMotor.stopMotor()
        interpolatedAimTable.putMap(Constants.PivotConstants.distanceMap)

        tab.addDouble("Relative Position") { getPosition() }
        tab.addDouble("Absolute Position") { getAbsEncoder() }
        tab.addBoolean("Encoders Set") { encoderSet }
        tab.addDouble("Voltage Sent") { pivotMotor.appliedOutput * pivotMotor.busVoltage }
    }

    private fun configureMotor(){
        pivotMotor.restoreFactoryDefaults()
        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        pivotMotor.burnFlash()
    }

    fun configureEncoders() {
        relativeEncoder.setPosition(0.0)
        relativeEncoder.positionConversionFactor = 1 / (Constants.PivotConstants.REL_ENCODER_CONVERSION)
    }

    fun zeroEncoder(){
        relativeEncoder.setPosition(0.0)
    }

    private fun configurePID(){
        // TODO: Figure out why lower values = more oscillation
        pidController.p = 50.0
        pidController.i = .1
        pidController.d = 0.0
//        pidController.setSmartMotionMaxAccel(5000.0, 0)
//        pidController.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0)
//        pidController.setSmartMotionMaxVelocity(5600.0, 0)
//        pidController.setSmartMotionMinOutputVelocity(0.0, 0)
//        pidController.setSmartMotionAllowedClosedLoopError(0.001, 0)
        pidController.setFeedbackDevice(relativeEncoder)
    }

    fun getAbsEncoder(): Double {
        return (absoluteEncoder.absolutePosition * Constants.PivotConstants.ABS_ENCODER_CONVERSION) - Constants.PivotConstants.ABSOLUTE_OFFSET
    }

    fun rawMotorSpeed(speed: Double){
        pivotMotor.set(speed)
        println("Raw motor speed is set to $speed")
    }

    fun setTargetPosition(position: Double){
        pidController.setReference(position, CANSparkBase.ControlType.kPosition)
        println("Set position to $position")
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

        if (Robot.isDisabled && encoderSet)
        {
            encoderSet = false
        }
    }



}