package lib.logging;

import dev.doglog.DogLog;

// Quite literally everything in this file must be package-private

/**
 * To spy on {@link DogLog}, this exists.
 */
final class DogLogSpy extends DogLog {
    private DogLogSpy() {
        throw new AssertionError("Do not instantiate DogLogSpy");
    }

    static boolean enabled() {
        return enabled;
    }
}
