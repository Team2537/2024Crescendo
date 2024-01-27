package lib.zones

import edu.wpi.first.math.geometry.Translation2d
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.decodeFromStream
import java.io.InputStream
import java.net.URL
import java.util.Spliterator
import java.util.function.Consumer
import java.util.stream.Stream
import kotlin.collections.ArrayList

/**
 * A singleton manager for [Zone]s
 *
 * On initialization, the list of zones is loaded from
 * [ZoneList.json](ZoneList.json)
 * by default, but it can be reloaded with different resources via
 * [loadZones]
 *
 * @author Matthew Clark
 *
 * @see Zone
 */
object Zones : List<Zone> {
    private val zoneList: MutableList<Zone>


    init {
        zoneList = ArrayList()
        loadZones(javaClass.getResourceAsStream("/ZoneList.json")!!)
    }

    @OptIn(ExperimentalSerializationApi::class)
    fun loadZones(stream: InputStream, doClose: Boolean = true){
        zoneList.clear()

        val zones = Json.decodeFromStream<Array<Zone>>(stream)

        zoneList.addAll(zones)

        if(doClose)
            stream.close()
    }

    fun loadZones(resource: URL, doClose: Boolean = true){
        loadZones(resource.openStream(), doClose)
    }

    /**
     * Gets a zone by its tag. If multiple zones are present, the first one found will
     * be returned. If no zone with the given tag is found, `null` will be returned.
     *
     * @param tag The tag to search with
     *
     * @return The zone with the given tag, or `null` if none are found
     */
    operator fun get(tag: String): Zone? {
        return zoneList.find { it.tag == tag }
    }

    /**
     * Gets the zone that the given position is inside. If multiple zones overlap,
     * the first one found will be returned. If the position is not inside a zone,
     * `null` will be returned.
     *
     * @param position The position on the field
     *
     * @return The zone the position is inside, or `null` if it is not inside a zone
     */
    operator fun get(position: Translation2d): Zone? {
        return zoneList.find { position in it }
    }

    override val size: Int
        get() = zoneList.size

    override fun containsAll(elements: Collection<Zone>): Boolean {
        return zoneList.containsAll(elements)
    }

    override operator fun contains(element: Zone): Boolean {
        return zoneList.contains(element)
    }

    override operator fun get(index: Int): Zone {
        return zoneList[index]
    }

    override fun isEmpty(): Boolean {
        return zoneList.isEmpty()
    }

    override fun iterator(): Iterator<Zone> {
        return zoneList.iterator()
    }

    override fun parallelStream(): Stream<Zone> {
        return zoneList.parallelStream()
    }

    override fun spliterator(): Spliterator<Zone> {
        return zoneList.spliterator()
    }

    override fun stream(): Stream<Zone> {
        return zoneList.stream()
    }

    override fun listIterator(): ListIterator<Zone> {
        return zoneList.listIterator()
    }

    override fun listIterator(index: Int): ListIterator<Zone> {
        return zoneList.listIterator(index)
    }

    override fun subList(fromIndex: Int, toIndex: Int): List<Zone> {
        return zoneList.subList(toIndex, fromIndex)
    }

    override fun lastIndexOf(element: Zone): Int {
        return zoneList.lastIndexOf(element)
    }

    override fun indexOf(element: Zone): Int {
        return zoneList.indexOf(element)
    }

    /**
     * Gets a zone by its tag, or returns a default if none are found
     *
     * @param tag The tag to search by
     * @param default The default value if no zone is found
     *
     * @return A non-null zone with given tag
     */
    fun getOrDefault(tag: String, default: Zone): Zone {
        return get(tag) ?: default
    }

    override fun equals(other: Any?): Boolean {
        return this === other
    }

    override fun hashCode(): Int {
        return zoneList.hashCode()
    }

    override fun toString(): String {
        return zoneList.toString()
    }

    override fun forEach(action: Consumer<in Zone>?) {
        zoneList.forEach(action)
    }
}