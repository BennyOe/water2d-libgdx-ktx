package com.github.bennyOe.water

import com.badlogic.gdx.physics.box2d.Body

/**
 * A single vertical water column with a spring-like update (tension + dampening).
 *
 * @param x world x position (anchor, immutable)
 * @param y world y position (anchor, immutable)
 * @param targetHeight equilibrium surface height
 * @param height current surface height (mutable)
 * @param speed current vertical speed of the column (mutable)
 */
class WaterColumn(
    val x: Float = 0f,
    val y: Float = 0f,
    val targetHeight: Float,
    var height: Float,
    var speed: Float
) {
    /** Optional Box2D body currently interacting with this column. */
    var actualBody: Body? = null

    /**
     * Updates the current height using a simple mass-spring-damper step.
     * @param dampening damping factor (0..1)
     * @param tension spring tension (>0)
     */
    fun update(dampening: Float, tension: Float) {
        val delta = targetHeight - height
        speed += tension * delta - speed * dampening
        height += speed
    }

    override fun toString(): String =
        "WaterColumn(x=$x, y=$y, target=$targetHeight, height=$height, speed=$speed)"
}
