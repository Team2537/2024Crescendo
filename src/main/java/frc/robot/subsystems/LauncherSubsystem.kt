import com.revrobotics.*
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants
import frc.robot.Robot
import lib.math.units.RotationVelocity
import lib.math.units.into
import lib.math.units.rpm
import lib.math.units.velocity
import lib.near
import lib.zoneTrigger

object LauncherSubsystem : SubsystemBase() {
    // TODO: Get the actual IDs
    val topFlywheels: CANSparkFlex = CANSparkFlex(Constants.LauncherConstants.TOP_FLYWHEELS,
        CANSparkLowLevel.MotorType.kBrushless)

    val bottomFlywheels: CANSparkFlex = CANSparkFlex(Constants.LauncherConstants.BOTTOM_FLYWHEELS,
        CANSparkLowLevel.MotorType.kBrushless)

    val rollerMotor: CANSparkMax = CANSparkMax(Constants.LauncherConstants.ROLLER_MOTOR,
        CANSparkLowLevel.MotorType.kBrushless)

//    val leftNoteDetector: DigitalInput = DigitalInput(Constants.LauncherConstants.LEFT_NOTE_DETECTOR)
    val rightNoteDetector: DigitalInput = DigitalInput(Constants.LauncherConstants.RIGHT_NOTE_DETECTOR)

    var setPoint: Double = 0.0

    val tab = Shuffleboard.getTab("Launcher")

    val flywheelPIDController: SparkPIDController = topFlywheels.pidController
    val flywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(
        0.0, 0.0, 0.0
    ) // TODO: Get real values


    val noteTrigger: Trigger

    val routine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> ->
                topFlywheels.setVoltage(volts.into(Units.Volts))
//                bottomFlywheels.setVoltage(volts.into(Units.Volts))
            },
            { log: SysIdRoutineLog ->
                log.motor("topFlywheels")
                    .voltage(Units.Volts.of(topFlywheels.appliedOutput * topFlywheels.busVoltage))
                    .angularPosition(Units.Rotations.of(topFlywheels.encoder.position))
                    .angularVelocity(Units.RPM.of(topFlywheels.encoder.velocity))
                    .current(Units.Amps.of(topFlywheels.outputCurrent))
//                log.motor("bottomFlywheels")
//                    .voltage(Units.Volts.of(bottomFlywheels.appliedOutput * bottomFlywheels.busVoltage))
//                    .angularPosition(Units.Rotations.of(bottomFlywheels.encoder.position))
//                    .angularVelocity(Units.RPM.of(bottomFlywheels.encoder.velocity))
//                    .current(Units.Amps.of(bottomFlywheels.outputCurrent))
            },
            this
        )
    )

    val topFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(-0.20832, 0.11109, 0.024896)

    init {
        topFlywheels.restoreFactoryDefaults()
        bottomFlywheels.restoreFactoryDefaults()
        rollerMotor.restoreFactoryDefaults()

        bottomFlywheels.inverted = true

        topFlywheels.setSmartCurrentLimit(40)
        bottomFlywheels.setSmartCurrentLimit(40)
        rollerMotor.setSmartCurrentLimit(40)

        topFlywheels.encoder.positionConversionFactor = 1.0
        bottomFlywheels.encoder.positionConversionFactor = 1.0
        topFlywheels.encoder.velocityConversionFactor = 1.0
        bottomFlywheels.encoder.velocityConversionFactor = 1.0

        topFlywheels.setIdleMode(CANSparkBase.IdleMode.kCoast)
        bottomFlywheels.setIdleMode(CANSparkBase.IdleMode.kCoast)


        rollerMotor.pidController.p = 0.25
        rollerMotor.pidController.i = 0.0
        rollerMotor.pidController.d = 0.0

        noteTrigger = Trigger() {
            !rightNoteDetector.get() && Robot.isEnabled
        }.debounce(0.1)

        Shuffleboard.getTab("Launcher").addBoolean("Note Detected") { noteTrigger.asBoolean }
        Shuffleboard.getTab("Launcher").addDouble("Position") { getRollerPosition() }
        Shuffleboard.getTab("Launcher").addDouble("Setpoint") { setPoint }

        Shuffleboard.getTab("Launcher Sysid").addDouble("Top Flywheel Position") { topFlywheels.encoder.position }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Top Flywheel Velocity") { topFlywheels.encoder.velocity }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Bottom Flywheel Position") { bottomFlywheels.encoder.position }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Bottom Flywheel Velocity") { bottomFlywheels.encoder.velocity }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Top Flywheel Voltage") {topFlywheels.appliedOutput * topFlywheels.busVoltage }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Bottom Flywheel Voltage") {bottomFlywheels.appliedOutput * bottomFlywheels.busVoltage }

        topFlywheels.burnFlash()
        bottomFlywheels.burnFlash()

        rollerMotor.encoder.setPosition(0.0)
        topFlywheels.encoder.setPosition(0.0)
        bottomFlywheels.encoder.setPosition(0.0)
    }

    fun setFlywheelSpeeds(rawSpeed: Double) {
        topFlywheels.set(rawSpeed)
        bottomFlywheels.set(rawSpeed)
    }

    fun setRollerSpeed(rawSpeed: Double) {
        rollerMotor.set(rawSpeed)
    }

    fun setFlywheelVelocity(velocity: RotationVelocity){
        topFlywheels.setVoltage(topFlywheelFeedforward.calculate(velocity into Units.RotationsPerSecond));
        bottomFlywheels.setVoltage(topFlywheelFeedforward.calculate(velocity into Units.RotationsPerSecond))
    }

    fun setFlywheelVoltage(voltage: Double){
        topFlywheels.setVoltage(voltage)
    }

    fun stopFlywheels() {
        topFlywheels.set(0.0)
        bottomFlywheels.set(0.0)
    }

    fun stopRoller() {
        rollerMotor.set(0.0)
    }

    fun setRollerPosition(position: Double) {
        rollerMotor.pidController.setReference(position, CANSparkBase.ControlType.kPosition)
        setPoint = position
    }

    fun getRollerPosition(): Double {
        return rollerMotor.encoder.position
    }

    // TODO: Add fancy logic
    fun noteDetected(): Boolean {
        return !rightNoteDetector.get()
    }

    fun dynamicSysIDRoutine(direction: Direction): Command? {
        return routine.dynamic(direction)
    }

    fun quasiStaticSysIDRoutine(direction: Direction): Command? {
        return routine.quasistatic(direction)
    }
}