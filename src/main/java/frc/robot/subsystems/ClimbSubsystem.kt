package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import frc.robot.Constants

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase

object ClimbSubsystem : SubsystemBase() {

    /** The spark max for the left motor */
    val leftMotor = CANSparkMax(Constants.ClimbConstants.LEFT_CLIMB_PORT, CANSparkLowLevel.MotorType.kBrushless)

    /** The spark max for the right motor */
    val rightMotor = CANSparkMax(Constants.ClimbConstants.RIGHT_CLIMB_PORT, CANSparkLowLevel.MotorType.kBrushless)

    init{
        // Conversion ratio for encoders, so that the position is in rotations outside of the gearbox
        leftMotor.encoder.setPositionConversionFactor(1.0/16.0)
        rightMotor.encoder.setPositionConversionFactor(1.0/16.0)

        // Logging for Shuffleboard, position and velocity of the motors
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Climb Subsystem")
        driveTab.addNumber("Left Climb Motor Velocity",) {leftMotor.encoder.velocity}
        driveTab.addNumber("Right Climb Motor Velocity",) {rightMotor.encoder.velocity}
        driveTab.addNumber("Left Climb Motor Position",) {leftMotor.encoder.position}
        driveTab.addNumber("Right Climb Motor Position",) {rightMotor.encoder.position}

        // Reset encoders to 0, since that only gets rest on power cycle not code deploy
        rightMotor.encoder.setPosition(0.0)
        leftMotor.encoder.setPosition(0.0)

        // Set the motors to brake mode so that they don't move (as easily) when disabled
        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)

        // "Burn" motor settings to the Spark Maxes, so that they persist across power cycles
        rightMotor.burnFlash()
        leftMotor.burnFlash()
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }

    /**
     * Raise the arms at a constant speed
     */
    fun armsUp() {
        leftMotor.set(Constants.ClimbConstants.MOTOR_SPEED_UP)
        rightMotor.set(Constants.ClimbConstants.MOTOR_SPEED_UP)
    }

    /**
     * Lower the arms at a constant speed
     */
    fun armsDown() {
        leftMotor.set(Constants.ClimbConstants.MOTOR_SPEED_DOWN)
        rightMotor.set(Constants.ClimbConstants.MOTOR_SPEED_DOWN)
    }

    /**
     * Stop the motors
     */
    fun stop() {
        leftMotor.set(0.0)
        rightMotor.set(0.0)
    }

    /**
     * Set the raw speeds of the motors
     * @param speed The speed to set the motors to
     */
    fun setRawSpeeds(speed: Double) {
        leftMotor.set(speed)
        rightMotor.set(speed)
    }
}