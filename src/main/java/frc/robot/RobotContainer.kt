package frc.robot

//import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.launcher.IntakeNoteCommand
import frc.robot.commands.pivot.QuickPivotCommand
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.subsystems.climb.ClimberSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.launcher.LaunchSubsystem
import frc.robot.subsystems.pivot.PivotSubsystem
import frc.robot.util.SingletonXboxController

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

    private val intakePivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.INTAKE_POSITION)
    private val ampPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.AMP_POSITION)
    private val speakerPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION)

    private val homePivot: Command = PivotSubsystem.homingRoutine()
    private val manualPivot = PivotSubsystem.manualPivot { controller.rightY }

    private val manualClimb = ClimberSubsystem.dualArmControl { controller.rightY }


    init {
        // TODO: comment stuff in this function cause I'm lazy (:
        initializeObjects()
        configureBindings()

        Shuffleboard.getTab("Scheduler").add("Scheduler", CommandScheduler.getInstance())

    }

    /**
     * Use to eager initialize objects
     */
    private fun initializeObjects() {
        ClimberSubsystem
        PivotSubsystem
        LaunchSubsystem
        IntakeSubsystem
        SwerveSubsystem
    }


    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Intake Command Group
        controller.leftBumper().toggleOnTrue(
            ParallelDeadlineGroup(
                IntakeNoteCommand(),
                IntakeSubsystem.intakeNote().alongWith(intakePivot)
            )
        )

        // Zero the gyro
        controller.leftStick().onTrue(InstantCommand(SwerveSubsystem::zeroGyro))
        // Toggle field oriented drive
        controller.rightStick().onTrue(InstantCommand(SwerveSubsystem::toggleFieldOriented))

        // Pivot position bindings
        controller.povUp().onTrue(ampPivot)
        controller.povDown().onTrue(speakerPivot)
        controller.povRight().onTrue(intakePivot)

        // Home the pivot
        controller.y().onTrue(homePivot)

        // Manual control bindings
        controller.b().toggleOnTrue(manualPivot)
        controller.x().toggleOnTrue(manualClimb)
    }


}
