package frc.robot.subsystems

import frc.robot.Constants

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase

object ClimbSubsystem : SubsystemBase() {
    val leftMotor = CANSparkMax(Constants.ClimbConstants.LEFT_CLIMB_PORT, CANSparkLowLevel.MotorType.kBrushless)
    val rightMotor = CANSparkMax(Constants.ClimbConstants.RIGHT_CLIMB_PORT, CANSparkLowLevel.MotorType.kBrushless)

    init{
        leftMotor.encoder.setPositionConversionFactor(1.0/16.0)
        rightMotor.encoder.setPositionConversionFactor(1.0/16.0)
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Climb Subsystem")
        driveTab.addNumber("Left Climb Motor Velocity",) {leftMotor.encoder.velocity}
        driveTab.addNumber("Right Climb Motor Velocity",) {rightMotor.encoder.velocity}
        driveTab.addNumber("Left Climb Motor Position",) {leftMotor.encoder.position}
        driveTab.addNumber("Right Climb Motor Position",) {rightMotor.encoder.position}

        rightMotor.encoder.setPosition(0.0)
        leftMotor.encoder.setPosition(0.0)
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }

    // Caller must be responsible for calling stop() when the command is finished
    fun armsUp() {
        leftMotor.set(Constants.ClimbConstants.MOTOR_SPEED_UP)
        rightMotor.set(Constants.ClimbConstants.MOTOR_SPEED_UP)
    }

    // Caller must be responsible for calling stop() when the command is finished
    fun armsDown() {
        leftMotor.set(Constants.ClimbConstants.MOTOR_SPEED_DOWN)
        rightMotor.set(Constants.ClimbConstants.MOTOR_SPEED_DOWN)
    }

    fun stop() {
        leftMotor.set(0.0)
        rightMotor.set(0.0)
    }

    fun setRawSpeeds(speed: Double) {
        leftMotor.set(speed)
        rightMotor.set(speed)
    }
}