package com.github.bennyOe.water

import com.badlogic.gdx.math.Vector2

/**
 * Particle representing a drop of water formed when a body impacts against water
 */
data class Particle(
    var position: Vector2,
    val velocity: Vector2,
    val radius: Float,
    var time: Float = 0f,
    val initX: Float = position.x
)

