import com.revrobotics.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import lib.math.units.RotationVelocity
import lib.math.units.rpm
import lib.math.units.velocity
import lib.near
import lib.zoneTrigger

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

    var fire: Boolean = false

    enum class State {
        STORED,
        EMPTY,
        PRIMED,
        AT_SPEED,
        FIRING
    }

    var state: State = State.EMPTY

    var storePosition: Double = getRollerPosition()
    val tab = Shuffleboard.getTab("Launcher")

    init {
        configureMotors()

        val irSensor: DigitalInput = DigitalInput(Constants.LauncherConstants.INFRARED_SENSOR)
        noteDetector = Trigger() { !irSensor.get() }
        noteTimer.start()

        tab.addBoolean("Note Detector") { noteDetector.asBoolean }
        tab.addString("State") { state.toString() }
        tab.addDouble("Timer") { noteTimer.get() }
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

    fun setLauncherSpeed(speed: Double){
        leftLauncher.set(speed)
    }

    fun getLauncherVelocity(): RotationVelocity {
        return leftLauncher.velocity
    }

    fun setRollerSpeed(speed: Double) {
        rollerMotor.set(speed)
    }

    fun stop() {
        setRollerMode(CANSparkBase.IdleMode.kBrake)

        leftLauncher.set(0.0)
        rollerMotor.set(0.0)
    }

    fun setRollerMode(idleMode: CANSparkBase.IdleMode){
        rollerMotor.setIdleMode(idleMode)
    }

    fun intake(){
        storePosition = getRollerPosition() - 0.5
        println("Intake ran, setting to $storePosition")
        rollerPID.setReference(storePosition, CANSparkBase.ControlType.kSmartMotion)
    }

    fun getRollerPosition(): Double {
        return rollerMotor.encoder.position
    }

    fun inZone(): Boolean {
        //return zoneTrigger("primeZone").asBoolean
        return true
    }

    fun updateState(){
        when(state){
            State.EMPTY -> {
                if(noteDetector.asBoolean){
                    state = State.STORED
                }
            }
            State.STORED -> {
                if(!noteDetector.asBoolean){
                    state = State.EMPTY
                } else if (noteDetector.asBoolean && storePosition.near(getRollerPosition(), 0.2) && inZone()){
                    state = State.PRIMED
                }
            }
            State.PRIMED -> {
                if(leftLauncher.velocity > Constants.LauncherConstants.MINIMUM_VELOCITY
                    && rightLauncher.velocity > Constants.LauncherConstants.MINIMUM_VELOCITY){
                    state = State.AT_SPEED
                } else if (!noteDetector.asBoolean) {
                    state = State.EMPTY
                }
            }
            State.AT_SPEED -> {
                if(fire){
                    fire = false
                    state = State.FIRING
                }
            }
            State.FIRING -> {
                if(noteTimer.hasElapsed(0.5)){
                    state = State.EMPTY
                    stop()
                }
            }
        }
    }

    fun triggerFactory(state: State): Trigger {
        return Trigger() { this.state == state}
    }

    override fun periodic() {
        if(noteDetector.asBoolean){
            noteTimer.reset()
        }

        updateState()
    }



}