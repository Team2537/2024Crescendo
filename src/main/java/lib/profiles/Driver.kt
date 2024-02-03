package lib.profiles

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import lib.controllers.ProfileController
import lib.profiles.Driver.loadControllers
import java.util.*
import java.util.function.Consumer
import java.util.stream.Stream

/**
 * A singleton manager for [controllers][ProfileController]
 *
 * The Driver manager keeps a list of all controllers that are in use.
 * The [ShuffleboardTab] named `"Controllers"` allows for the changing of
 * each controllers' profile at any time. The profile choices are obtained
 * through [DriverProfile.Profile].
 *
 * By default, no controllers are present in the Driver object (this should
 * be changed), but controllers can be loaded through [loadControllers]
 *
 * @author Matthew Clark
 *
 * @since 2024-02-03
 */
object Driver : List<ProfileController> {
    // list impl is subject to change
    private val controllers: MutableList<ProfileController> = ArrayList()

    override val size: Int
        get() = controllers.size

    private fun reloadShuffleboard(){
        val tab = Shuffleboard.getTab("Controllers")

        val chooser = SendableChooser<ProfileController>().apply {
            forEach { addOption(it.name, it) }
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
            chooser.selected.profile = it
        }

        // Proper information in case the choosers get outdated selection (see to-do above)
        tab.addString("Active") { "${chooser.selected.name}: ${chooser.selected.profile}" }

        tab.add(chooser)

        tab.add(profileChooser)
    }

    /**
     * Loads controllers into this driver from a [Stream].
     *
     * This will be terminal to the stream provided, as it makes use of
     * [Stream.forEach]
     *
     * @param controllers The stream to load from
     * @param doClear Whether to clear the existing controllers before adding
     */
    fun loadControllers(controllers: Stream<out ProfileController>, doClear: Boolean = true){
        if(doClear)
            this.controllers.clear()

        controllers.forEach { this.controllers.add(it) }

        reloadShuffleboard()
    }

    /**
     * Loads controllers into this driver.
     *
     * @param controllers The controllers to add
     * @param doClear Whether to clear the existing controllers
     */
    fun loadControllers(controllers: Collection<ProfileController>, doClear: Boolean = true){
        if(doClear)
            this.controllers.clear()
        this.controllers.addAll(controllers)

        reloadShuffleboard()
    }

    /**
     * Loads controllers into this driver.
     *
     * @param controllers The controllers to add
     * @param doClear Whether to clear the existing controllers
     */
    fun loadControllers(controllers: Array<out ProfileController>, doClear: Boolean = true){
        if(doClear)
            this.controllers.clear()
        this.controllers.addAll(controllers)

        reloadShuffleboard()
    }


    /** @suppress */
    override fun get(index: Int): ProfileController {
        return controllers[index]
    }

    /** @suppress */
    override fun isEmpty(): Boolean {
        return controllers.isEmpty()
    }

    /** @suppress */
    override fun iterator(): Iterator<ProfileController> {
        return controllers.iterator()
    }


    /** @suppress */
    override fun listIterator(): ListIterator<ProfileController> {
        return controllers.listIterator()
    }


    /** @suppress */
    override fun listIterator(index: Int): ListIterator<ProfileController> {
        return controllers.listIterator(index)
    }


    /** @suppress */
    override fun subList(fromIndex: Int, toIndex: Int): List<ProfileController> {
        return controllers.subList(fromIndex, toIndex)
    }


    /** @suppress */
    override fun lastIndexOf(element: ProfileController): Int {
        return controllers.lastIndexOf(element)
    }


    /** @suppress */
    override fun indexOf(element: ProfileController): Int {
        return controllers.indexOf(element)
    }

    /** @suppress */
    override fun containsAll(elements: Collection<ProfileController>): Boolean {
        return controllers.containsAll(elements)
    }

    /** @suppress */
    override fun contains(element: ProfileController): Boolean {
        return controllers.contains(element)
    }

    /** @suppress */
    override fun spliterator(): Spliterator<ProfileController> {
        return controllers.spliterator()
    }

    /** @suppress */
    override fun hashCode(): Int {
        return controllers.hashCode()
    }

    /** @suppress */
    override fun parallelStream(): Stream<ProfileController> {
        return controllers.parallelStream()
    }

    /** @suppress */
    override fun stream(): Stream<ProfileController> {
        return controllers.stream()
    }

    /** @suppress */
    override fun toString(): String {
        return controllers.toString()
    }

    /** @suppress */
    override fun forEach(action: Consumer<in ProfileController>?) {
        controllers.forEach(action)
    }
}