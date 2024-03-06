package lib.swerve

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units
import lib.math.units.Rotation
import lib.math.units.Span
import lib.math.units.into
import lib.setLoopRampRate
import lib.setPosition
import lib.setVelocity

class Module {
    val driveMotor: CANSparkMax
    val angleMotor: CANSparkMax
    val angleEncoder: RelativeEncoder
    val driveEncoder: RelativeEncoder
    val absoluteEncoder: CANcoder
    val position: Translation2d
    val offset: Double

    constructor(
        driveID: Int,
        angleID: Int,
        absoluteID: Int,
        position: Translation2d,
        offset: Double,
        driveInverted: Boolean,
        angleInverted: Boolean,
        pid: ModulePID
    ) {
        driveMotor = CANSparkMax(driveID, CANSparkLowLevel.MotorType.kBrushless)
        angleMotor = CANSparkMax(angleID, CANSparkLowLevel.MotorType.kBrushless)
        absoluteEncoder = CANcoder(absoluteID)
        driveEncoder = driveMotor.encoder
        angleEncoder = angleMotor.encoder
        this.position = position
        this.offset = offset

        // Configure Motors

        driveMotor.restoreFactoryDefaults()
        angleMotor.restoreFactoryDefaults()

        driveMotor.pidController.p = pid.driveKp
        driveMotor.pidController.i = pid.driveKi
        driveMotor.pidController.d = pid.driveKd
        driveMotor.pidController.ff = pid.driveKf
        driveMotor.pidController.iZone = pid.driveIZone

        angleMotor.pidController.p = pid.angleKp
        angleMotor.pidController.i = pid.angleKi
        angleMotor.pidController.d = pid.angleKd
        angleMotor.pidController.ff = pid.angleKf
        angleMotor.pidController.iZone = pid.angleIZone

        driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        angleMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)

        driveMotor.inverted = driveInverted
        angleMotor.inverted = angleInverted

        driveMotor.setLoopRampRate(0.25)
        angleMotor.setLoopRampRate(0.25)

        driveMotor.setSmartCurrentLimit(40)
        angleMotor.setSmartCurrentLimit(40)

        driveMotor.enableVoltageCompensation(12.0)
        angleMotor.enableVoltageCompensation(12.0)

        driveMotor.burnFlash()
        angleMotor.burnFlash()

        // Configure Encoders
        absoluteEncoder.setPosition(absoluteEncoder.absolutePosition.valueAsDouble - offset)
        angleEncoder.setPosition(getAbsolutePosition())
    }

    constructor(
        driveID: Int,
        angleID: Int,
        absoluteID: Int,
        xPos: Span,
        yPos: Span,
        offset: Double,
        driveInverted: Boolean,
        angleInverted: Boolean,
        pid: ModulePID
    ) : this(
        driveID,
        angleID,
        absoluteID,
        Translation2d(xPos into Units.Meters, yPos into Units.Meters),
        offset,
        driveInverted,
        angleInverted,
        pid
    )

    fun getAbsolutePosition(): Double {
        return absoluteEncoder.position.valueAsDouble
    }

    fun zeroAbsolutePosition() {
        absoluteEncoder.setPosition(absoluteEncoder.absolutePosition.valueAsDouble - offset)
    }

    fun zero() {
        driveEncoder.setPosition(0.0)
        setAngle(0.0)
    }

    fun setAngle(angle: Double) {
        angleMotor.setPosition(angle)
    }

    fun resetEncoders() {
        driveEncoder.setPosition(0.0)
        angleEncoder.setPosition(getAbsolutePosition())
    }

    fun getDriveVoltage(): Double {
        return driveMotor.appliedOutput * driveMotor.busVoltage
    }

    fun getAngle(): Double {
        return angleEncoder.position
    }

    // If not working, make sure passed in states are desaturated
    fun setDesiredState(state: SwerveModuleState) {
        val state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()))
        driveMotor.setVelocity(state.speedMetersPerSecond)
        setAngle(state.angle.degrees)
    }





}




