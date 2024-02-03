package lib.profiles

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import lib.controllers.ProfileController
import lib.controllers.XboxGamepad
import lib.profiles.Driver.loadControllers
import java.util.*

/**
 * A singleton manager for [controllers][ProfileController]
 *
 * The Driver manager keeps a list of all controllers that are in use.
 * The [ShuffleboardTab] named `"Controllers"` allows for the changing of
 * each controllers' profile at any time. The profile choices are obtained
 * through [DriverProfile.Profile].
 *
 * By default, an [XboxGamepad] on port 0 with a default profile is the only
 * controller, but controllers can be loaded through [loadControllers]
 *
 * @author Matthew Clark
 *
 * @since 2024-02-03
 */
object Driver : Iterable<ProfileController> {
    // list impl is subject to change
//    private val controllers: MutableList<ProfileController> = ArrayList()

    // I have been forced to hard-code this. I hate it.
    private var controller0: ProfileController
    private var controller1: ProfileController? = null
    private var controller2: ProfileController? = null
    private var controller3: ProfileController? = null

    private var controllerChooser: SendableChooser<ProfileController>

    private val tab: ShuffleboardTab

    /**
     * The number of controllers that can exist.
     *
     * @return 4
     */
    val size: Int
        get() = 4

    init {
        controller0 = ProfileController(XboxGamepad(0))

        tab = Shuffleboard.getTab("Controllers")
        controllerChooser = SendableChooser<ProfileController>().apply {
            addOption("Controller 0", controller0)
            addOption("Controller 1", controller1)
            addOption("Controller 2", controller2)
            addOption("Controller 3", controller3)
            //setDefaultOption(controllers[0].name, controllers[0])
        }
        val profileChooser = SendableChooser<DriverProfile.Profile>().apply {
            DriverProfile.Profile.entries.forEach {
                addOption(it.friendlyName, it)
            }
        }
        // TODO: I'd prefer if the selected profile switched to the one active on the controller
        //  when the controller is switched, but I don't know how to do that or if it is even
        //  possible
        profileChooser.onChange {
            controllerChooser.selected.profile = it
        }
        // Proper information in case the choosers get outdated selection (see to-do above)
        tab.addString("Active") { "${controllerChooser.selected?.name}: ${controllerChooser.selected?.profile}" }
        tab.add(controllerChooser)
        tab.add(profileChooser)
    }

    /**
     * Loads controllers into this driver.
     *
     * @param controller0 controller0
     * @param controller1 controller1
     * @param controller2 controller2
     * @param controller3 controller3
     */
    fun loadControllers(
        controller0: ProfileController,
        controller1: ProfileController? = null,
        controller2: ProfileController? = null,
        controller3: ProfileController? = null,
    ) {
        this.controller0 = controller0
        this.controller1 = controller1
        this.controller2 = controller2
        this.controller3 = controller3
    }


    /** @suppress */
    operator fun get(index: Int): ProfileController {
        // lol
        return when(index){
            0 -> controller0
            1 -> controller1 ?: controller0
            2 -> controller2 ?: controller1 ?: controller0
            3 -> controller3 ?: controller2 ?: controller1 ?: controller0
            else -> throw IndexOutOfBoundsException("Bad index: $index")
        }
    }

    /** @suppress */
    operator fun set(index: Int, value: ProfileController) {
        when(index){
            0 -> controller0 = value
            1 -> controller1 = value
            2 -> controller2 = value
            3 -> controller3 = value
            else -> throw IndexOutOfBoundsException("Bad index: $index")
        }
    }

    /** @suppress */
    override fun iterator(): Iterator<ProfileController> {
        return Spliterators.iterator(spliterator())
    }

    /** @suppress */
    fun indexOf(controller: ProfileController): Int {
        return when(controller){
            controller0 -> 0
            controller1 -> 1
            controller2 -> 2
            controller3 -> 3
            else -> -1
        }
    }

    /** @suppress */
    operator fun contains(controller: ProfileController): Boolean {
        return when(controller){
            controller0, controller1, controller2, controller3 -> true
            else -> false
        }
    }

    /** @suppress */
    override fun spliterator(): Spliterator<ProfileController> {
        return Spliterators.spliterator(arrayOf(controller0, controller1, controller2, controller3), 0)
    }

    /** @suppress */
    override fun hashCode(): Int {
        return Objects.hash(controller0, controller1, controller2, controller3)
    }

    /** @suppress */
    override fun toString(): String {
        return "[$controller0, $controller1, $controller2, $controller3]"
    }
}