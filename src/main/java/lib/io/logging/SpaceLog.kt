package lib.io.logging

import dev.doglog.DogLog
import edu.wpi.first.util.struct.StructSerializable
import lib.io.logging.SpaceLog.Level
import java.util.*

/**
 * A custom logger built for FRC Team 2537 "Space Raiders" that wraps
 * [DogLog](https://doglog.dev/).
 *
 * This implementation offers logging [levels][Level] with runtime mutation.
 * This allows for real-time observation for debugging purposes and reducing
 * overhead for less commonly needed messages.
 *
 * @author Matthew Clark
 *
 * @see DogLog
 */
object SpaceLog {
    /**
     * A level at which the [SpaceLog] can log.
     */
    enum class Level {
        /**
         * The finest level of logging, for tracing robot behaviour.
         */
        TRACE,

        /**
         * A level for general debug information.
         */
        DEBUG,

        /**
         * The default level. For information that is almost always useful to have.
         */
        INFO,

        /**
         * A level for warnings about non-fatal incidents that are best avoided.
         */
        WARN,

        /**
         * A level for errors, whether fatal or not, that absolutely must be recorded.
         */
        ERROR
    }

    /**
     * The [Level] at which [SpaceLog] logs.
     */
    @JvmStatic
    @Volatile
    @set:Synchronized
    var level: Level = Level.INFO


    private fun willLog(lvl: Level): Boolean {
        return level <= lvl
    }

    // Logs
    // TODO: add corresponding log messages to all data types supported by DogLog

    /**
     * Logs a message under a given key at a given [Level]. Will not log if the
     * logger's level is set above the level given.
     *
     * @param lvl The level to log at.
     * @param key The key to log under.
     * @param msg The message to log.
     */
    @JvmStatic
    fun log(lvl: Level, key: String, msg: String) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", msg)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, long: Long) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", long)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: LongArray) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, int: Int) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", int.toLong())
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: IntArray) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, short: Short) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", short.toLong())
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: ShortArray) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr.contentToString())
    }

    @JvmStatic
    fun log(lvl: Level, key: String, byte: Byte) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", byte.toLong())
    }

    @JvmStatic
    fun log(lvl: Level, key: String, char: Char) {
        if (!willLog(lvl)) return
        // This format of characters may change if it is revealed that it sucks.
        DogLog.log("$key/$lvl", "'$char' (\\u${Integer.toHexString(char.code)})")
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: ByteArray) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr.contentToString())
    }

    @JvmStatic
    fun log(lvl: Level, key: String, double: Double) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", double)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: DoubleArray) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr)
    }
    @JvmStatic
    fun log(lvl: Level, key: String, float: Float) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", float)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: FloatArray) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, bool: Boolean) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", bool)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: BooleanArray) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, enum: Enum<*>) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", enum)
    }

    @JvmStatic
    fun log(lvl: Level, key: String, obj: Any?) {
        if (!willLog(lvl)) return

        // The extra method overhead is not needed for a one object case
        if(obj is StructSerializable)
            DogLog.log("$key/$lvl", obj)
        else
            DogLog.log("$key/$lvl", obj.toString())
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: Array<out Any?>) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr.contentToString())
    }

    @JvmStatic
    fun log(lvl: Level, key: String, arr: Array<out StructSerializable>) {
        if (!willLog(lvl)) return
        DogLog.log("$key/$lvl", arr)
    }


    /**
     * Logs a message under a given key at [trace level][Level.TRACE]. Will not
     * log if the logger's level is higher than trace.
     *
     * @param key The key to log under.
     * @param msg The message to log.
     */
    @JvmStatic
    fun trace(key: String, msg: String) {
        log(Level.TRACE, key, msg)
    }

    /**
     * Logs a message under a given key at [debug level][Level.DEBUG]. Will not
     * log if the logger's level is higher than debug.
     *
     * @param key The key to log under.
     * @param msg The message to log.
     */
    @JvmStatic
    fun debug(key: String, msg: String) {
        log(Level.DEBUG, key, msg)
    }

    /**
     * Logs a message under a given key at [info level][Level.INFO]. Will not
     * log if the logger's level is higher than info.
     *
     * @param key The key to log under.
     * @param msg The message to log.
     */
    @JvmStatic
    fun info(key: String, msg: String) {
        log(Level.TRACE, key, msg)
    }

    /**
     * Logs a message under a given key at [warning level][Level.WARN]. Will not
     * log if the logger's level is higher than warn.
     *
     * @param key The key to log under.
     * @param msg The message to log.
     */
    @JvmStatic
    fun warn(key: String, msg: String) {
        log(Level.WARN, key, msg)
    }

    /**
     * Logs a message under a given key at [error level][Level.ERROR]. Will always
     * log, and will log a fault to DogLog.
     *
     * @param key The key to log under.
     * @param msg The message to log.
     */
    @JvmStatic
    fun error(key: String, msg: String) {
        log(Level.ERROR, key, msg)
        DogLog.logFault("Error logged: $msg")
    }

    /**
     * Logs an error with its cause. Will log a fault to DogLog.
     *
     * @param key The key to log under.
     * @param msg The message to log.
     * @param cause The cause of the error. This will be appended to the
     * logged message.
     */
    @JvmStatic
    fun error(key: String, msg: String, cause: Throwable) {
        error(key, "$msg\nCaused by: $cause")
    }

    /**
     * Logs an error with its cause. Will log a fault to DogLog.
     *
     * @param key The key to log under.
     * @param cause The cause of the error. This will be used for the
     * logged message.
     */
    @JvmStatic
    fun error(key: String, cause: Throwable) {
        if (cause.cause != null) {
            error(key, cause.message ?: "", cause.cause!!)
        } else {
            error(key, cause.message ?: "")
        }
    }

    // Log format
    // TODO: versions of format logs for all of the above

    /**
     * Logs a message under a given key at a given [Level]. Will not log nor
     * format the given string if the logger is set to ta level above that given.
     *
     * @param lvl The level to log at.
     * @param key The key to log under.
     * @param fmt The string format to log.
     * @param args The arguments to format into the given string.
     */
    @JvmStatic
    @Suppress("SpellCheckingInspection")
    fun logf(lvl: Level, key: String, fmt: String, vararg args: Any?) {
        // Additional check for enabled; this check is ommitted elsewhere because we
        // must format before sending to DogLog (unless we create a custum struct
        // that will format lazily (which I just might do)).
        if (!willLog(lvl) || !DogLogSpy.enabled()) return

        DogLog.log(key, format(fmt, args))
    }

    private fun format(fmt: String, args: Array<out Any?>): String {
        // FIXME: Implement a format of choice
        //  String#format would work for C-style formatting, but more modern
        //  choices should be considered.
//        throw UnsupportedOperationException("Unimplemented Method: SpaceLog#format(String, Object...)")
        // asterisk spreads array into varargs
        return String.format(fmt, *args)
    }
}
