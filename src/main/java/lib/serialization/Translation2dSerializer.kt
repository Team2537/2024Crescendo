package lib.serialization

import edu.wpi.first.math.geometry.Translation2d
import kotlinx.serialization.*
import kotlinx.serialization.descriptors.*
import kotlinx.serialization.encoding.*

/**
 * Custom *explicitly defined* serializer for [Translation2d]
 *
 * @author Matthew Clark
 */
@Suppress("EXTERNAL_SERIALIZER_USELESS")
@OptIn(ExperimentalSerializationApi::class)
@Serializer(forClass = Translation2d::class)
class Translation2dSerializer : KSerializer<Translation2d> {
    /**
     * Describes the structure of the serializable representation of [KSerializer], produced
     * by this serializer. Knowing the structure of the descriptor is required to determine
     * the shape of the serialized form (e.g. what elements are encoded as lists and what as primitives)
     * along with its metadata such as alternative names.
     *
     * The descriptor is used during serialization by encoders and decoders
     * to introspect the type and metadata of [KSerializer]'s elements being encoded or decoded, and
     * to introspect the type, infer the schema or to compare against the predefined schema.
     */
    @OptIn(InternalSerializationApi::class)
    override val descriptor: SerialDescriptor
        get() = buildSerialDescriptor(
            "Translation2d",
            StructureKind.CLASS
        ){
            element<Double>("x")
            element<Double>("y")
        }

    override fun deserialize(decoder: Decoder): Translation2d =
        decoder.decodeStructure(descriptor){
            var x: Double = 0.0
            var y: Double = 0.0

            do {
                when(val index = decodeElementIndex(descriptor)){
                    0 -> x = decodeDoubleElement(descriptor, index)
                    1 -> y = decodeDoubleElement(descriptor, index)
                    CompositeDecoder.DECODE_DONE -> break
                    else -> throw SerializationException("Unexpected index: $index")
                }
            } while(true)

            return@decodeStructure Translation2d(x, y)
        }

    /**
     * Indicates whether some other object is "equal to" this one. Implementations must fulfil the following
     * requirements:
     *
     * * Reflexive: for any non-null value `x`, `x.equals(x)` should return true.
     * * Symmetric: for any non-null values `x` and `y`, `x.equals(y)` should return true if and only if `y.equals(x)` returns true.
     * * Transitive:  for any non-null values `x`, `y`, and `z`, if `x.equals(y)` returns true and `y.equals(z)` returns true, then `x.equals(z)` should return true.
     * * Consistent:  for any non-null values `x` and `y`, multiple invocations of `x.equals(y)` consistently return true or consistently return false, provided no information used in `equals` comparisons on the objects is modified.
     * * Never equal to null: for any non-null value `x`, `x.equals(null)` should return false.
     *
     * Read more about [equality](https://kotlinlang.org/docs/reference/equality.html) in Kotlin.
     */
    override fun equals(other: Any?): Boolean {
        return super.equals(other)
    }

    /**
     * Returns a hash code value for the object.  The general contract of `hashCode` is:
     *
     * * Whenever it is invoked on the same object more than once, the `hashCode` method must consistently return the same integer, provided no information used in `equals` comparisons on the object is modified.
     * * If two objects are equal according to the `equals()` method, then calling the `hashCode` method on each of the two objects must produce the same integer result.
     */
    override fun hashCode(): Int {
        return super.hashCode()
    }

    /**
     * Serializes the [value] of type [T] using the format that is represented by the given [encoder].
     * [serialize] method is format-agnostic and operates with a high-level structured [Encoder] API.
     * Throws [SerializationException] if value cannot be serialized.
     *
     * Example of serialize method:
     * ```
     * class MyData(int: Int, stringList: List<String>, alwaysZero: Long)
     *
     * fun serialize(encoder: Encoder, value: MyData): Unit = encoder.encodeStructure(descriptor) {
     *     // encodeStructure encodes beginning and end of the structure
     *     // encode 'int' property as Int
     *     encodeIntElement(descriptor, index = 0, value.int)
     *     // encode 'stringList' property as List<String>
     *     encodeSerializableElement(descriptor, index = 1, serializer<List<String>>, value.stringList)
     *     // don't encode 'alwaysZero' property because we decided to do so
     * } // end of the structure
     * ```
     *
     * @throws SerializationException in case of any serialization-specific error
     * @throws IllegalArgumentException if the supplied input does not comply encoder's specification
     * @see KSerializer for additional information about general contracts and exception specifics
     */
    override fun serialize(encoder: Encoder, value: Translation2d) {
        encoder.encodeStructure(descriptor){
            encodeDoubleElement(descriptor, 0,  value.x)
            encodeDoubleElement(descriptor, 1,  value.y)
        }
    }

    /**
     * Returns a string representation of the object.
     */
    override fun toString(): String {
        return super.toString()
    }

}