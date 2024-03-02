package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants

object ClimbSubsystem : SubsystemBase() {
    val leftArm: CANSparkMax
    val rightArm: CANSparkMax

    init {
        leftArm = CANSparkMax(ClimbConstants.LEFT_ARM_ID, CANSparkLowLevel.MotorType.kBrushless)
        rightArm = CANSparkMax(ClimbConstants.RIGHT_ARM_ID, CANSparkLowLevel.MotorType.kBrushless)

        leftArm.restoreFactoryDefaults()
        rightArm.restoreFactoryDefaults()

        leftArm.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rightArm.setIdleMode(CANSparkBase.IdleMode.kBrake)
    }

    fun setArmSpeeds(speed: Double){
        leftArm.set(speed)
        rightArm.set(speed)
    }
}