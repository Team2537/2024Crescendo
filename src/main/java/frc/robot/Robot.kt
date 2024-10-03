package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.swerve.Drivebase
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import kotlin.jvm.optionals.getOrDefault

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : LoggedRobot() {

    //    val climb = Climb()
//    val pivot = Pivot()
    val drivebase = Drivebase()
//    val intake = Intake()
//    val launcher = Launcher()


    val driverController: CommandXboxController = CommandXboxController(0).apply {
        // BINDINGS GO HERE
        drivebase.defaultCommand = drivebase.driveCommand(
            { -leftY },
            { -leftX },
            { -rightX },
            leftBumper()
        )
    }

    private val routines: AutoRoutines = AutoRoutines(drivebase.factory)

    init {
        Logger.recordMetadata("Project Name", "2024Crescendo")

        when (Constants.RobotConstants.mode) {
            Constants.RobotConstants.Mode.REAL -> {
                Logger.recordMetadata("Mode", "Real")
//                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
                PowerDistribution(1, PowerDistribution.ModuleType.kRev)
            }

            Constants.RobotConstants.Mode.SIM -> {
                Logger.recordMetadata("Mode", "Sim")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.RobotConstants.Mode.REPLAY -> {
                setUseTiming(false)
                Logger.recordMetadata("Mode", "Replay")
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")))
            }
        }

        Logger.start()

        DriverStation.silenceJoystickConnectionWarning(true)
    }


    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }

    /** This method is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {
    }

    override fun disabledPeriodic() {
    }

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        routines.selectedRoutine.schedule()
    }

    /** This method is called periodically during autonomous.  */
    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {
        routines.selectedRoutine.cancel()
    }

    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This method is called periodically during test mode.  */
    override fun testPeriodic() {
    }

    /** This method is called once when the robot is first started up.  */
    override fun simulationInit() {
    }

    /** This method is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {
    }
}
