package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.LauncherConstants
import lib.math.units.Rotation
import lib.math.units.RotationVelocity
import lib.math.units.into
import lib.math.units.velocity

object LauncherSubsystem : SubsystemBase() {

    val angleMotor = CANSparkMax(LauncherConstants.ANGLE_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless)
    val storeMotor: CANSparkMax = CANSparkMax(LauncherConstants.STORE_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless)
    val leftLauncherMotor : CANSparkFlex = CANSparkFlex(LauncherConstants.LEFT_LAUNCHER_PORT, CANSparkLowLevel.MotorType.kBrushless)
    val rightLauncherMotor : CANSparkFlex = CANSparkFlex(LauncherConstants.RIGHT_LAUNCHER_PORT, CANSparkLowLevel.MotorType.kBrushless)


    val launchPID: SparkPIDController = leftLauncherMotor.pidController
    val storePID: SparkPIDController = storeMotor.pidController
    val anglePID: SparkPIDController = angleMotor.pidController

    val launchEncoder: RelativeEncoder = leftLauncherMotor.encoder
    val storeEncoder: RelativeEncoder = storeMotor.encoder

    val relativeAngleEncoder: RelativeEncoder = angleMotor.encoder
    val absoluteAngleEncoder: DutyCycleEncoder = DutyCycleEncoder(LauncherConstants.ABSOLUTE_ENCODER_PORT)

    val irPieceSensor = DigitalInput(Constants.LauncherConstants.IR_PIECE_SENSOR_ID)


    init {
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Launcher Subsystem")

        anglePID.p = LauncherConstants.ANGLE_KP
        anglePID.i = LauncherConstants.ANGLE_KI
        anglePID.d = LauncherConstants.ANGLE_KD
      
        launchPID.p = LauncherConstants.LAUNCHER_P
        launchPID.i = LauncherConstants.LAUNCHER_I

        rightLauncherMotor.follow(leftLauncherMotor, true)


        driveTab.addNumber("Motor1 Velocity") {
            leftLauncherMotor.velocity into RPM
            rightLauncherMotor.velocity into RPM
        }

        driveTab.addNumber("Motor Velocity") {
            angleMotor.velocity into RPM
        }

    }

    fun setAngle(angle: Rotation) {
        anglePID.setReference(angle into Rotations, CANSparkBase.ControlType.kPosition)
    }
    
    fun setRawLaunchSpeed(rA: Double, rB: Double) {
        leftLauncherMotor.set(rA)
    }

    fun setLaunchVelocity(velocity: RotationVelocity) {
        leftLauncherMotor.velocity = velocity
    }
    
    fun intake() {
        storePID.setReference(0.5, CANSparkBase.ControlType.kPosition)
    }
    val piecePresent: Boolean
        get() = irPieceSensor.get()
    fun outtake() {
        storeMotor.set(0.5)
    }

    override fun periodic() {}
}