package frc.robot

import LauncherSubsystem
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.Autos
import frc.robot.commands.intake.FeedLauncherCommand
import frc.robot.commands.intake.ManualIntakeCommand
import frc.robot.commands.intake.ToggleIntakeCommand
import frc.robot.commands.launcher.*
import frc.robot.commands.pivot.HoldPositionCommand
import frc.robot.commands.pivot.ManualPivotCommand
import frc.robot.commands.pivot.QuickPivotCommand
import frc.robot.commands.swerve.*
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.PivotSubsystem
import frc.robot.subsystems.SwerveSubsystem
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



//    val trackTarget = TrackTargetCommand()

    // TODO: This is kinda weird but inverting (and the drive encoders) makes it display properly
    //     No, uninverting both doesn't fix it :(
    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { MathUtil.applyDeadband(-controller.leftY, 0.1) },
            { MathUtil.applyDeadband(-controller.leftX, 0.1) },
            { MathUtil.applyDeadband(-controller.rightX, 0.1)},
            { controller.hid.leftBumper },
            { controller.hid.rightBumper },
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

    val pullNoteCommand: PullNoteCommand = PullNoteCommand()

    val intakePivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.INTAKE_POSITION)
    val launcherPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION)
    val ampPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.AMP_POSITION)

    val manualPivot: ManualPivotCommand = ManualPivotCommand() { controller.rightY }




    init {
        // TODO: comment stuff in this function cause I'm lazy (:
        initializeObjects()
        configureBindings()

        SwerveSubsystem.defaultCommand = teleopDrive

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
        controller.leftStick().onTrue(InstantCommand(SwerveSubsystem::zeroGyro))
        controller.rightStick().toggleOnTrue(correctedDrive)
        controller.y().onTrue(InstantCommand(PivotSubsystem::zeroEncoder))
        controller.pov(0).onTrue(ampPivot)
        controller.pov(180).onTrue(launcherPivot)
        controller.pov(90).onTrue(intakePivot)
        controller.pov(270).toggleOnTrue(HoldPositionCommand())
        controller.x().toggleOnTrue(ToggleIntakeCommand().alongWith(pullNoteCommand))
        controller.leftTrigger().onTrue(ReadyFireCommand())
//        controller.b().toggleOnTrue(manualPivot)
        controller.b().onTrue(DriftTestCommand(2.0, 1.0))
        stateBindings()
    }


    private fun stateBindings(){
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.EMPTY)
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.STORED).whileTrue(IntakeCommand())
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.PRIMED).whileTrue(PrimeLauncherCommand())
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.AT_SPEED)
            .and(controller.leftTrigger()).onTrue(ReadyFireCommand())
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.FIRING).whileTrue(FireCommand().alongWith(FeedLauncherCommand()))
    }

    private fun addNamedCommands() {
        NamedCommands.registerCommand("Intake", PrintCommand("Intake"))
        NamedCommands.registerCommand("Launch", PrintCommand("Launch"))
    }
}
