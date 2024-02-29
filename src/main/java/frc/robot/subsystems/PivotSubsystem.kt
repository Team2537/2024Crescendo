package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants
import frc.robot.Robot

object PivotSubsystem : SubsystemBase() {
    val pivotMotor: CANSparkMax = CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless)

    val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(PivotConstants.ABSOLUTE_ENCODER_PORT)
    val relativeEncoder: RelativeEncoder = pivotMotor.encoder
    val pivotPID: SparkPIDController = pivotMotor.pidController

    val tab: ShuffleboardTab = Shuffleboard.getTab("Pivot")

    var encoderSet: Boolean = false

    private val feedforward: ArmFeedforward = ArmFeedforward(
        PivotConstants.kS,
        PivotConstants.kG,
        PivotConstants.kV,
        PivotConstants.kA
    )

    init {
        pivotMotor.restoreFactoryDefaults()
        relativeEncoder.positionConversionFactor = PivotConstants.REL_ENCODER_CONVERSION
        absoluteEncoder.distancePerRotation = PivotConstants.ABS_ENCODER_CONVERSION
        absoluteEncoder.positionOffset = PivotConstants.ABSOLUTE_OFFSET

        pivotPID.p = PivotConstants.kP
        pivotPID.i = PivotConstants.kI
        pivotPID.d = PivotConstants.kD
        pivotPID.ff = 0.0

        tab.addDouble("Throughbore Distance") { getAbsolutePosition() }
        tab.addDouble("Absolute Position") { absoluteEncoder.absolutePosition }
        tab.addDouble("Relative Position") { getRelativePosition() }
        tab.addDouble("Voltage Sent") { pivotMotor.appliedOutput * pivotMotor.busVoltage }

        zeroEncoder()

        pivotMotor.setSmartCurrentLimit(40)
    }

    fun getAbsolutePosition(): Double {
        return absoluteEncoder.distance
    }

    fun getRelativePosition(): Double {
        return relativeEncoder.position
    }

    fun syncRelative(){
        relativeEncoder.setPosition(getAbsolutePosition())
    }

    fun zeroEncoder() {
        relativeEncoder.setPosition(0.0)
    }

    fun setRawSpeed(speed: Double){
        pivotMotor.set(speed)
    }
    fun setVoltage(voltage: Double) {
        pivotMotor.setVoltage(voltage)
    }

    fun resetEncoder() {
        absoluteEncoder.reset()
    }

    fun setPIDPosition(position: Double) {
        pivotPID.setReference(position, CANSparkBase.ControlType.kPosition)
        println("Setting position to $position")
    }

    fun holdArm(position: Double){
        pivotPID.setReference(position,
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.calculate(Units.degreesToRadians(position), 0.0)
        )
    }

    fun stop() {
        pivotMotor.stopMotor()
    }

    override fun periodic() {
//        if(!encoderSet && Robot.isEnabled){
//            syncRelative()
//            encoderSet = true
//        } else if (encoderSet && Robot.isDisabled) {
//            encoderSet = false
//        }
    }

}