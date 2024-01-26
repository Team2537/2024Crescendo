package frc.robot.subsystems

import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants

object LauncherSubsystem : SubsystemBase() {
    val angleMotor = CANSparkFlex(Constants.LauncherConstants.ANGLE_MOTOR, CANSparkLowLevel.MotorType.kBrushless)


    private val kP = Constants.LauncherConstants.ANGLE_KP
    private val kI = Constants.LauncherConstants.ANGLE_KI
    private val kD = Constants.LauncherConstants.ANGLE_KD
    private val pidController = PIDController(kP, kI, kD)

    init {
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Launcher Subsystem")
        val pid = angleMotor.pidController
        pid.p=kP
        pid.i=kI
        pid.d=kD

        driveTab.addNumber("Motor Velocity")
        {
            angleMotor.encoder.velocity
        }

        fun set(rA: Double){
            angleMotor.set(rA)
        }
        fun setAngle(pos: Double){
            pidController.setpoint = pos
        }
    }
    override fun periodic() {}
}