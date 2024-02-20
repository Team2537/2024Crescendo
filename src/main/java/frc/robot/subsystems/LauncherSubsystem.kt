import com.revrobotics.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants

object LauncherSubsystem : SubsystemBase() {

    private val leftLauncher: CANSparkFlex = CANSparkFlex(Constants.LauncherConstants.LEFT_LAUNCHER_PORT,
        CANSparkLowLevel.MotorType.kBrushless)

    private val rightLauncher: CANSparkFlex = CANSparkFlex(Constants.LauncherConstants.RIGHT_LAUNCHER_PORT,
        CANSparkLowLevel.MotorType.kBrushless)

    private val rollerMotor: CANSparkMax = CANSparkMax(Constants.LauncherConstants.ROLLER_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless)

    private val rollerPID: SparkPIDController = rollerMotor.pidController

    private val noteDetector: Trigger

    private val noteTimer: Timer = Timer()

    enum class State {
        STORED,
        EMPTY,
        PRIMED,
        AT_SPEED,
        FIRING
    }

    var state: State = State.EMPTY
        get() = this.state

    init {
        configureMotors()

        val irSensor: DigitalInput = DigitalInput(Constants.LauncherConstants.INFRARED_SENSOR)
        noteDetector = Trigger() { !irSensor.get() }
        noteTimer.start()
    }


    private fun configureMotors(){
        leftLauncher.restoreFactoryDefaults()
        rightLauncher.restoreFactoryDefaults()
        rollerMotor.restoreFactoryDefaults()

        rightLauncher.follow(leftLauncher, true)

        configureEncoders()
        configurePID()

        leftLauncher.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rightLauncher.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
    }

    private fun configurePID() {
        rollerPID.p = Constants.LauncherConstants.ROLLER_P
        rollerPID.i = Constants.LauncherConstants.ROLLER_I
        rollerPID.d = 0.0

        rollerPID.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0)
        rollerPID.setSmartMotionMaxVelocity(240.0, 0)
        rollerPID.setSmartMotionMinOutputVelocity(0.0, 0)
        rollerPID.setSmartMotionAllowedClosedLoopError(0.05, 0)
        rollerPID.setSmartMotionMaxAccel(1000.0, 0)
    }

    private fun configureEncoders() {
        leftLauncher.encoder.positionConversionFactor = 1.0
        leftLauncher.encoder.velocityConversionFactor = 1.0

        rightLauncher.encoder.positionConversionFactor = 1.0
        rightLauncher.encoder.velocityConversionFactor = 1.0

        rollerMotor.encoder.positionConversionFactor = 1.0
        rollerMotor.encoder.velocityConversionFactor = 1.0
    }


}