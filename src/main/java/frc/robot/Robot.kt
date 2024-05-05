package frc.robot

import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.Autos
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

    /**
     * The autonomous command to run. While a default value is set here,
     * the [autonomousInit] method will set it to the value selected in
     *the  AutoChooser on the dashboard.
     */
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        Logger.recordMetadata("ProjectName", "2024Crescendo")
        Logger.recordMetadata("RobotName", Constants.RobotData.ROBOT_NAME)
        Logger.recordMetadata("GitSha", GitConstants.GIT_SHA)
        Logger.recordMetadata("GitBranch", GitConstants.GIT_BRANCH)
        Logger.recordMetadata("BuildTime", GitConstants.BUILD_DATE)
        Logger.recordMetadata("Revision Count", GitConstants.GIT_REVISION.toString())
        when(GitConstants.DIRTY){
            1 -> Logger.recordMetadata("GitDirty", "Uncommited Changes")
            0 -> Logger.recordMetadata("GitDirty", "No Uncommited Changes")
            else -> Logger.recordMetadata("GitDirty", "Unknown")
        }


        when (Constants.RobotData.MODE) {
            Constants.RobotData.Mode.REAL -> {
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.RobotData.Mode.SIM -> {
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.RobotData.Mode.REPLAY -> {
                val logPath: String = LogFileUtil.findReplayLog() // This pulls from AdvantageScope among other things
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")))
                Logger.addDataReceiver(NT4Publisher())
            }
        }

//        if (isReal()) {
//            Logger.addDataReceiver(WPILOGWriter())
//            Logger.addDataReceiver(NT4Publisher())
//        } else {
////            setUseTiming(false)
////            val logPath: String = LogFileUtil.findReplayLog()
////            Logger.setReplaySource(WPILOGReader(logPath))
////            Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")))
//            Logger.addDataReceiver(NT4Publisher())
//        }

        Logger.start()

        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        RobotContainer
        enableLiveWindowInTest(true)
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
        Autos.selectedAutonomousCommand.cancel()
        // We store the command as a Robot property in the rare event that the selector on the dashboard
        // is modified while the command is running since we need to access it again in teleopInit()
        Autos.selectedAutonomousCommand.schedule()
    }

    /** This method is called periodically during autonomous.  */
    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running. If you want the
        // autonomous to continue until interrupted by another command, remove this line or comment it out.
        Autos.selectedAutonomousCommand.cancel()
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
