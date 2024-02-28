package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants

object PivotSubsystem : SubsystemBase() {
    val pivotMotor: CANSparkMax = CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless)

    val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(PivotConstants.ABSOLUTE_ENCODER_PORT)
    val relativeEncoder: RelativeEncoder = pivotMotor.encoder
    val pivotPID: SparkPIDController = pivotMotor.pidController

    val tab: ShuffleboardTab = Shuffleboard.getTab("Pivot")

    val feedforward: ArmFeedforward = ArmFeedforward(
        PivotConstants.kS,
        PivotConstants.kG,
        PivotConstants.kV,
        PivotConstants.kA
    )

    init {
        pivotMotor.restoreFactoryDefaults()
        relativeEncoder.positionConversionFactor = PivotConstants.REL_ENCODER_CONVERSION
        absoluteEncoder.distancePerRotation = PivotConstants.ABS_ENCODER_CONVERSION

        pivotPID.p = PivotConstants.kP
        pivotPID.i = PivotConstants.kI
        pivotPID.d = PivotConstants.kD
        pivotPID.ff = 0.0

        tab.addDouble("Absolute Encoder") { getAbsolutePosition() }
        tab.addDouble("Relative Position") { getRelativePosition() }
        tab.addDouble("Voltage Sent") { pivotMotor.appliedOutput * pivotMotor.busVoltage }
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

    fun setRawSpeed(speed: Double){
        pivotMotor.set(speed)
    }
    fun setVoltage(voltage: Double) {
        pivotMotor.setVoltage(voltage)
    }

    fun resetEncoder() {
        absoluteEncoder.reset()
    }


}