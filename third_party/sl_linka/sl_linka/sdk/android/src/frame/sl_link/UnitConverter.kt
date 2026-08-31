package sl_link

/**
 * Small helpers for the current SL-LinkA control model.
 */
object UnitConverter {
    fun clampSpeedRatio(value: Double): Float = value.coerceIn(0.0, 1.0).toFloat()
}
