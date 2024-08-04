package frc.robot.subsystems

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoControlFunction
import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants
import frc.robot.Constants.Drivebase.maxAngularVelocity
import frc.robot.Constants.Drivebase.maximumSpeed
import lib.math.units.into
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.SwerveModule
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.util.function.DoubleSupplier

class SwerveSubsystem : SubsystemBase() {

    /** YAGSL's Swerve drive object for hardware interaction */
    private val drivebase: SwerveDrive

    val board: ShuffleboardTab = Shuffleboard.getTab("Swerve Data")

    var isFieldOriented: Boolean = true

    val kinematics: SwerveDriveKinematics
        get() = drivebase.kinematics

    val pose: Pose2d
        get() = drivebase.pose

    val heading: Rotation2d
        get() = drivebase.yaw

    val fieldVelocity: ChassisSpeeds
        get() = drivebase.fieldVelocity

    val robotVelocity: ChassisSpeeds
        get() = drivebase.robotVelocity

    val controller: SwerveController
        get() = drivebase.swerveController

    val config: SwerveDriveConfiguration
        get() = drivebase.swerveDriveConfiguration

    val choreoController: ChoreoControlFunction =
        Choreo.choreoSwerveController(
            Constants.Auto.xAutoPID,
            Constants.Auto.yAutoPID,
            Constants.Auto.thetaAutoPID,
        )


    init {
        // MAKE SURE TO CHANGE THIS
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

        try {
            drivebase = SwerveParser(Constants.FileConstants.CRESCENDO_CONFIG)
                .createSwerveDrive(maximumSpeed.into(Units.MetersPerSecond))
        } catch (e: Exception) {
            e.printStackTrace()
            throw RuntimeException("Error initializing SwerveDrive.", e)
        }

        drivebase.setHeadingCorrection(true)

        if (SwerveDriveTelemetry.isSimulation) {
            drivebase.setCosineCompensator(false)
        } else {
            drivebase.setCosineCompensator(true)
        }
    }

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        drivebase.setChassisSpeeds(chassisSpeeds)
    }

    fun driveCommand(
        xSpeed: DoubleSupplier,
        ySpeed: DoubleSupplier,
        thetaSpeed: DoubleSupplier
    ): Command {
        return this.run { drive(xSpeed, ySpeed, thetaSpeed) }
    }

    fun toggleFieldOriented(): Command {
        return this.runOnce { isFieldOriented = !isFieldOriented }
    }

    private fun drive(xSpeed: DoubleSupplier, ySpeed: DoubleSupplier, thetaSpeed: DoubleSupplier) {
        val translation = Translation2d(
            maximumSpeed.into(Units.MetersPerSecond) * xSpeed.asDouble,
            maximumSpeed.into(Units.MetersPerSecond) * ySpeed.asDouble
        )

        drivebase.drive(
            translation,
            maxAngularVelocity.into(Units.RadiansPerSecond) * thetaSpeed.asDouble,
            isFieldOriented,
            false
        )
    }

    fun zeroGyro(){
        drivebase.zeroGyro()
    }

    private fun getDriveSysID(): SysIdRoutine {
        return SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { volts: Measure<Voltage> ->
                    drivebase.modules.forEach { module ->
                        module.driveMotor.voltage = volts.into(Units.Volts)
                    }
                },
                { log: SysIdRoutineLog ->
                    drivebase.modules.forEach {
                        logDriveMotor(it, log)
                    }
                },
                this
            )
        )
    }

    private fun getAngleSysID(): SysIdRoutine {
        return SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { volts: Measure<Voltage> ->
                    drivebase.modules.forEach { module ->
                        module.angleMotor.voltage = volts.into(Units.Volts)
                    }
                },
                { log: SysIdRoutineLog ->
                    drivebase.modules.forEach {
                        logAngleMotor(it, log)
                    }
                },
                this
            )
        )
    }

    fun getDriveSysIDDynamic(direction: SysIdRoutine.Direction): Command {
        return this.run {
            SwerveDriveTest.centerModules(drivebase)
            getDriveSysID().dynamic(direction)
        }
    }

    fun getDriveSysIDQuasistatic(direction: SysIdRoutine.Direction): Command {
        return this.run {
            SwerveDriveTest.centerModules(drivebase)
            getDriveSysID().quasistatic(direction)
        }
    }

    fun getAngleSysIDDynamic(direction: SysIdRoutine.Direction): Command {
        return this.run {
            getAngleSysID().dynamic(direction)
        }
    }

    fun getAngleSysIDQuasistatic(direction: SysIdRoutine.Direction): Command {
        return this.run {
            getAngleSysID().quasistatic(direction)
        }
    }

    /**
     * Logging function to easily log for SysID.
     * @see SysIdRoutineLog
     * @param module The module to log.
     * @param log The SysIdRoutineLog to log to.
     */
    private fun logDriveMotor(module: SwerveModule, log: SysIdRoutineLog) {
        log.motor(module.configuration.name)
            .voltage(Units.Volt.of(module.driveMotor.voltage))
            .linearPosition(Units.Meters.of(module.driveMotor.position))
            .linearVelocity(Units.MetersPerSecond.of(module.driveMotor.velocity))
    }

    private fun logAngleMotor(module: SwerveModule, log: SysIdRoutineLog) {
        log.motor(module.configuration.name)
            .voltage(Units.Volt.of(module.angleMotor.voltage))
            .angularPosition(Units.Degrees.of(module.angleMotor.position))
            .angularVelocity(Units.DegreesPerSecond.of(module.angleMotor.velocity))
    }

    fun resetOdometry(newPose: Pose2d) {
        drivebase.resetOdometry(newPose)
    }
}