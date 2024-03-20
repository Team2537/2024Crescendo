package frc.robot

import LauncherSubsystem
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.Autos
import frc.robot.commands.ClimbArmsUpCommand
import frc.robot.commands.climb.ClimbToTargetCommand
import frc.robot.commands.climb.ManualClimbCommand
import frc.robot.commands.intake.FeedLauncherCommand
import frc.robot.commands.intake.ManualIntakeCommand
import frc.robot.commands.intake.ToggleIntakeCommand
import frc.robot.commands.launcher.*
import frc.robot.commands.pivot.*
import frc.robot.commands.swerve.*
import frc.robot.commands.vision.UpdateOdometryCommand
import frc.robot.subsystems.*
//import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.SingletonXboxController
import lib.profiles.DriverProfile

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
    // Testing pre-commit

    private val controller = SingletonXboxController // TODO: refactor to use ProfileController

    val launcherIsUsed: Trigger = Trigger() { CommandScheduler.getInstance().requiring(LauncherSubsystem) == null }


//    val trackTarget = TrackTargetCommand()

    // TODO: This is kinda weird but inverting (and the drive encoders) makes it display properly
    //     No, uninverting both doesn't fix it :(
    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { MathUtil.applyDeadband(-controller.leftY, 0.1) },
            { MathUtil.applyDeadband(-controller.leftX, 0.1) },
            { MathUtil.applyDeadband(-controller.rightX, 0.1)},
            { controller.rightTriggerAxis },
        )

    val correctedDrive: CorrectedDriveCommand =
        CorrectedDriveCommand(
            { MathUtil.applyDeadband(-controller.leftY, 0.1) },
            { MathUtil.applyDeadband(-0.0, 0.1) },
            { MathUtil.applyDeadband(-controller.rightX, 0.1)},
            { controller.hid.leftBumper },
            { controller.hid.rightBumper },
        )

    val cornerSpin: CornerSpinCommand =
        CornerSpinCommand(
            { -controller.rightX },
            { controller.hid.leftBumper },
            { controller.hid.rightBumper },
        )

    val absoluteDrive: AbsoluteDriveCommand = AbsoluteDriveCommand(
        { -controller.leftY },
        { -controller.leftX },
        { -controller.rightX },
        { -controller.rightY }
    )

    val manualIntake: ManualIntakeCommand = ManualIntakeCommand(
        { -controller.rightTriggerAxis },
        { controller.leftTriggerAxis }
    )

    val intakePivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.INTAKE_POSITION, false, false)
    val subwooferPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION, false, false)
    val ampPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.AMP_POSITION, false, false)
    val podiumPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.MID_POSITION, false, false)
    val autoAim: QuickPivotCommand = QuickPivotCommand(0.0, false, true)

    val manualPivot: ManualPivotCommand = ManualPivotCommand() { controller.rightY }

    val manualClimb: ManualClimbCommand = ManualClimbCommand() { controller.rightY }
    val climbCommand: ClimbToTargetCommand = ClimbToTargetCommand(7.0)

    val launchCommand: LaunchCommand = LaunchCommand(
        {1.0},
        { controller.leftTrigger(0.75).asBoolean },
        { PivotSubsystem.getRelativePosition() },
        { controller.button(Constants.OperatorConstants.START_BUTTON).asBoolean }
    )

    val testFlywheelSpeedsCommand: TestFlywheelSpeedsCommand = TestFlywheelSpeedsCommand() { controller.rightTriggerAxis }






    init {
        // TODO: comment stuff in this function cause I'm lazy (:
        initializeObjects()
        configureBindings()

        SwerveSubsystem.defaultCommand = teleopDrive

        Shuffleboard.getTab("Scheduler").add("Scheduler", CommandScheduler.getInstance())

    }

    /**
     * Use to eager initialize objects
     */
    private fun initializeObjects() {
        Autos
//        SwerveSubsystem
        Autos
//        LimelightSubsystem
        DriverProfile
        PivotSubsystem
        LauncherSubsystem
        IntakeSubsystem
        ClimbSubsystem
    }

    // Replace with CommandPS4Controller or CommandJoystick if needed

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        controller.leftBumper().toggleOnTrue(
            ParallelDeadlineGroup(
                IntakeNoteCommand(),
                ToggleIntakeCommand()
            )
        )
        controller.leftStick().onTrue(InstantCommand(SwerveSubsystem::zeroGyro))
        controller.povUp().onTrue(ampPivot)
        controller.povRight().onTrue(intakePivot)
        controller.povDown().onTrue(subwooferPivot)
        controller.povLeft().onTrue(autoAim) // TODO: Implement auto-aiming
        controller.y().onTrue(HomePivotCommand()) // TODO: Implement homing launcher
        controller.b().toggleOnTrue(manualPivot)
        controller.x().onTrue(manualClimb)
//        controller.rightBumper().toggleOnTrue(manualClimb)
        controller.rightStick().onTrue(InstantCommand(SwerveSubsystem::toggleFieldOriented))
        controller.button(Constants.OperatorConstants.BACK_BUTTON)
            .onTrue(TestFlywheelFeedforwardCommand())
//        controller.button(Constants.OperatorConstants.START_BUTTON)
//            .whileTrue(SequentialCommandGroup(
//                LauncherSubsystem.dynamicSysIDRoutine(SysIdRoutine.Direction.kForward),
//                WaitCommand(3.0),
//                LauncherSubsystem.dynamicSysIDRoutine(SysIdRoutine.Direction.kReverse),
//                WaitCommand(3.0),
//                LauncherSubsystem.quasiStaticSysIDRoutine(SysIdRoutine.Direction.kForward),
//                WaitCommand(3.0),
//                LauncherSubsystem.quasiStaticSysIDRoutine(SysIdRoutine.Direction.kReverse)
//            ))


        LauncherSubsystem.noteTrigger.and(controller.a()).onTrue(launchCommand) // TODO: Implement Priming



    }


}
