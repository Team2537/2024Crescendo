package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase

object ClimbSubsystem : SubsystemBase() {
    val leftMotor = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless)
    val rightMotor = CANSparkMax (5, CANSparkLowLevel.MotorType.kBrushless)

    init{
        leftMotor.encoder.setPositionConversionFactor(16.0/1.0)
        rightMotor.encoder.setPositionConversionFactor(16.0/1.0)
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Drive Subsystem")
        driveTab.addNumber("Left Climb Motor Velocity",) {return@addNumber leftMotor.encoder.velocity }
        driveTab.addNumber("Right Climb Motor Velocity",) {return@addNumber rightMotor.encoder.velocity}
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }
    }
