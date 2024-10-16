package frc.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.swerve.Drivebase
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.superstructure.pivot.Pivot
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

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

    // This is so awful, but it's the best way to test DIO in simulation that I can think of
    val keyboard: Joystick by lazy { println("JOYSTICK INITIALIZED"); Joystick(5) }

    val climb = Climb()
    val pivot = Pivot()
    val drivebase = Drivebase()
    val intake = Intake()
//    val launcher = Launcher()

    val robotPose
        get() = drivebase.pose

    val driverController: CommandXboxController = CommandXboxController(0)
    val operatorController: CommandXboxController = CommandXboxController(1)

    init {
        DriverStation.silenceJoystickConnectionWarning(true)

        Logger.recordMetadata("Project Name", "2024Crescendo")

        when(Constants.RobotConstants.mode){
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

        CommandScheduler.getInstance().onCommandInitialize { command ->
            Logger.recordOutput("commands/${command.name}", true)
        }

        CommandScheduler.getInstance().onCommandFinish { command ->
            Logger.recordOutput("commands/${command.name}", false)
        }

        CommandScheduler.getInstance().onCommandInterrupt { command ->
            Logger.recordOutput("commands/${command.name}", false)
        }

        Logger.start()
        configureBindings()
    }

    private fun configureBindings() {
        drivebase.defaultCommand = drivebase.driveCommand(
            { -driverController.leftY },
            { -driverController.leftX },
            { -driverController.rightX },
            driverController.leftBumper()
        )

        driverController.rightBumper().onTrue(InstantCommand({ drivebase.resetHeading() }))

        operatorController.a().onTrue(
            either(
                sequence(
                    pivot.getSendToPositionCommand(Pivot.intakePosition),
                    parallel(
                        intake.getEjectCommand(),
                        print("Ejecting Launcher") // Replace with actual launcher eject command
                    )
                ),
                sequence(
                    pivot.getSendToPositionCommand(Pivot.intakePosition),
                    parallel(
                        intake.getIntakeCommand(),
                        print("Intaking Launcher") // Replace with actual launcher intake command
                    )
                ),
                intake.isFull
            ).withName("Intake Auto Command")
        )

        operatorController.y().onTrue(pivot.getHomeCommand())

        operatorController.b().and(climb.isPreclimb).onTrue(climb.getExtendCommand())
        operatorController.b().and(climb.isExtended).onTrue(climb.getRetractCommand())
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
    override fun autonomousInit() {}

    /** This method is called periodically during autonomous.  */
    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {

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
