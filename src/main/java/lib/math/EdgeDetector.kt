package lib.math

class EdgeDetector(var value: Boolean = false) {
    var risingEdge = false
        private set
    var fallingEdge = false
        private set
    val changed: Boolean
        get() = risingEdge || fallingEdge

    fun update(value: Boolean) {
        risingEdge = false
        fallingEdge = false
        if (value && !this.value) {
            risingEdge = true
        }
        if (!value && this.value) {
            fallingEdge = true
        }
        this.value = value
    }
}