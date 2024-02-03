package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants

object LauncherSubsystem : SubsystemBase() {
    val angleMotor = CANSparkMax(LauncherConstants.ANGLE_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless)

    val storeMotor: CANSparkMax = CANSparkMax(LauncherConstants.STORE_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless)
  
    val leftLauncherMotor : CANSparkFlex = CANSparkFlex(LauncherConstants.LEFT_LAUNCHER_PORT, CANSparkLowLevel.MotorType.kBrushless)
    val rightLauncherMotor : CANSparkFlex = CANSparkFlex(LauncherConstants.RIGHT_LAUNCHER_PORT, CANSparkLowLevel.MotorType.kBrushless)


    val launchPID = leftLauncherMotor.pidController
    val storePID = storeMotor.pidController
    val anglePID = angleMotor.pidController


    init {
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Launcher Subsystem")


        anglePID.p = LauncherConstants.ANGLE_KP
        anglePID.i = LauncherConstants.ANGLE_KI
        anglePID.d = LauncherConstants.ANGLE_KD
      
        launchPID.p = LauncherConstants.LAUNCHER_P
        launchPID.i = LauncherConstants.LAUNCHER_I

        rightLauncherMotor.follow(leftLauncherMotor, true)


        driveTab.addNumber("Motor1 Velocity") {
            leftLauncherMotor.encoder.velocity
            rightLauncherMotor.encoder.velocity
        }

        driveTab.addNumber("Motor Velocity") {
            angleMotor.encoder.velocity
        }

    }

    fun setAngle(angle: Double) {
        anglePID.setReference(angle, CANSparkBase.ControlType.kPosition)
    }
    
    fun setRawLaunchSpeed(rA: Double, rB: Double) {
        leftLauncherMotor.set(rA)
    }

    fun setLaunchVelocity(velocity: Double){
        launchPID.setReference(velocity, CANSparkBase.ControlType.kVelocity)
    }
    
    fun intake(){
        
    }


    override fun periodic() {}
}