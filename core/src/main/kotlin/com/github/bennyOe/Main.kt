package com.github.bennyOe

import com.badlogic.gdx.ApplicationAdapter
import com.badlogic.gdx.Gdx
import com.badlogic.gdx.Input
import com.badlogic.gdx.graphics.Camera
import com.badlogic.gdx.graphics.GL20
import com.badlogic.gdx.graphics.OrthographicCamera
import com.badlogic.gdx.math.Vector2
import com.badlogic.gdx.math.Vector3
import com.badlogic.gdx.physics.box2d.*
import com.badlogic.gdx.scenes.scene2d.Stage
import com.badlogic.gdx.utils.viewport.StretchViewport
import com.github.bennyOe.water.Water

class Main : ApplicationAdapter() {

    companion object {
        val GRAVITY = Vector2(0f, -10f)
        private const val TIME_STEP = 1f / 60f
        private const val VEL_ITERS = 6
        private const val POS_ITERS = 2

        // Pixels-per-meter scale used across camera & input conversions
        private const val PPM = 100f
    }

    private lateinit var stage: Stage
    private lateinit var camera: OrthographicCamera
    private lateinit var world: World
    private lateinit var water: Water
    private lateinit var debugRenderer: Box2DDebugRenderer

    // Reusable temp vectors to avoid per-frame allocations
    private val tmp3 = Vector3()
    private val tmp2 = Vector2()

    override fun create() {
        camera = OrthographicCamera()
        stage = StretchViewport(Gdx.graphics.width / PPM, Gdx.graphics.height / PPM, camera).let(::Stage)

        world = World(GRAVITY, true).apply {
            setContactListener(MyContactListener())
        }

        debugRenderer = Box2DDebugRenderer()

        water = Water().apply {
            // world, x, y, width, height
            createBody(world, 3f, 0f, 8f, 2f)
            // isDebugMode = true
        }
    }

    override fun render() {
        // Clear screen
        Gdx.gl.glClearColor(0f, 0f, 0f, 1f)
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT)

        // Input handling
        if (Gdx.input.justTouched()) {
            createBodyAtInput() // idiomatic screen→world conversion inside
        }
        if (Gdx.input.isKeyJustPressed(Input.Keys.D)) {
            water.isDebugMode = !water.isDebugMode
        }

        // Physics step
        world.step(TIME_STEP, VEL_ITERS, POS_ITERS)

        // Water update & draw
        water.update()
        water.draw(camera as Camera)

        // Debug draw
        debugRenderer.render(world, camera.combined)
    }

    override fun dispose() {
        water.dispose()
        world.dispose()
        debugRenderer.dispose()
        stage.dispose()
    }

    // --- Helpers ------------------------------------------------------------------------------

    private fun createBodyAtInput() {
        val worldPos = screenToWorld(Gdx.input.x, Gdx.input.y)

        val bodyDef = BodyDef().apply {
            type = BodyDef.BodyType.DynamicBody
            position.set(worldPos)
        }
        val body = world.createBody(bodyDef)

        val square = PolygonShape().apply {
            setAsBox(0.3f, 0.3f) // world units already
        }

        val fixtureDef = FixtureDef().apply {
            shape = square
            density = 0.5f
            friction = 0.5f
            restitution = 0.5f
        }

        body.createFixture(fixtureDef)
        square.dispose()
    }

    /**
     * Converts screen coordinates (pixels) to world coordinates (meters).
     * Reuses tmp vectors to avoid allocations.
     */
    private fun screenToWorld(screenX: Int, screenY: Int): Vector2 {
        tmp3.set(screenX.toFloat(), screenY.toFloat(), 0f)
        camera.unproject(tmp3) // accounts for viewport transform & y inversion
        return tmp2.set(tmp3.x, tmp3.y)
    }
}
