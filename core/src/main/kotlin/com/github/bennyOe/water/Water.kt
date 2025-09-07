package com.github.bennyOe.water

import com.badlogic.gdx.Gdx
import com.badlogic.gdx.graphics.Camera
import com.badlogic.gdx.graphics.Texture
import com.badlogic.gdx.graphics.g2d.PolygonRegion
import com.badlogic.gdx.graphics.g2d.PolygonSprite
import com.badlogic.gdx.graphics.g2d.PolygonSpriteBatch
import com.badlogic.gdx.graphics.g2d.SpriteBatch
import com.badlogic.gdx.graphics.g2d.TextureRegion
import com.badlogic.gdx.graphics.glutils.ShapeRenderer
import com.badlogic.gdx.graphics.glutils.ShapeRenderer.ShapeType
import com.badlogic.gdx.math.EarClippingTriangulator
import com.badlogic.gdx.math.GeometryUtils
import com.badlogic.gdx.math.Vector2
import com.badlogic.gdx.physics.box2d.Body
import com.badlogic.gdx.physics.box2d.BodyDef
import com.badlogic.gdx.physics.box2d.BodyDef.BodyType
import com.badlogic.gdx.physics.box2d.Fixture
import com.badlogic.gdx.physics.box2d.FixtureDef
import com.badlogic.gdx.physics.box2d.PolygonShape
import com.badlogic.gdx.physics.box2d.World
import com.badlogic.gdx.utils.Disposable
import com.github.bennyOe.Main.Companion.GRAVITY
import com.github.bennyOe.water.IntersectionUtils.getRandomVector
import java.util.LinkedHashSet

import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.random.Random

private const val MIN_SPLASH_AREA: Float = 0.1f
private const val DRAG_MOD: Float = 0.25f
private const val LIFT_MOD: Float = 0.25f
private const val MAX_DRAG: Float = 2000f
private const val MAX_LIFT: Float = 500f
private const val TORQUE_DAMPING = 100f

/**
 * Allows to create an object to simulate the behavior of water in interaction with other bodies
 */
class Water(
    private val waves: Boolean = true,
    private val splashParticles: Boolean = true
) : Disposable {
    var isDebugMode: Boolean = false

    var spriteBatch: SpriteBatch? = null
    var polyBatch: PolygonSpriteBatch? = null
    var shapeBatch: ShapeRenderer?
    var textureWater: TextureRegion? = null
    var textureDrop: Texture? = null

    var fixturePairs: MutableSet<Pair<Fixture, Fixture>> = LinkedHashSet() // contacts between this object and other dynamic bodies
    var columns: MutableList<WaterColumn> = mutableListOf() // represent the height of the waves
    var particles: MutableList<Particle> = mutableListOf() // splash particles
    private var body: Body? = null
    private var graphicsReady = false

    var tension: Float = 0.025f
    var dampening: Float = 0.025f
    var spread: Float = 0.25f
    var density: Float = 1f

    val columnSeparation: Float = 0.02f // 4 px between every column

    /**
     * Constructor that allows to specify if there is an effect of waves and splash particles.
     *
     * @param waves           Specifies whether the object will have waves
     * @param splashParticles Specifies whether the object will have splash particles
     */

    init {
        this.isDebugMode = false

        // Defer all GPU/Gdx-dependent initialization until after Application is created
        textureWater = null
        polyBatch = null
        shapeBatch = null
        textureDrop = null
        spriteBatch = null
        if (splashParticles) {
            particles = mutableListOf()
        }
    }

    // Lazily initialize GPU/Gdx resources after Application is created
    private fun ensureGraphicsInitialized() {
        // Idempotent: skip if already initialized
        if (graphicsReady) return

        // Shape renderer is always needed for debug and particle boxes
        shapeBatch = shapeBatch ?: ShapeRenderer().also { it.setColor(0f, 0.5f, 1f, 1f) }

        if (waves) {
            polyBatch = polyBatch ?: PolygonSpriteBatch()
            textureWater = textureWater ?: TextureRegion(Texture(Gdx.files.internal("water.png")))
        }
        if (splashParticles) {
            spriteBatch = spriteBatch ?: SpriteBatch()
            textureDrop = textureDrop ?: Texture(Gdx.files.internal("drop.png"))
        }

        graphicsReady = true
    }

    /**
     * Creates the body of the water. It will be a square sensor in a specific Box2d world.
     *
     * @param world  Our box2d world
     * @param centerX      Position of the x coordinate of the center of the body
     * @param centerY      Position of the y coordinate of the center of the body
     * @param width  Body width
     * @param height Body height
     */
    fun createBody(world: World, centerX: Float, centerY: Float, width: Float, height: Float) {
        // Guard: dimensions must be positive
        require(width > 0f && height > 0f) { "Water area must be positive." }

        // Create static body at (centerX, centerY)
        val b = world.createBody(BodyDef().apply {
            type = BodyType.StaticBody
            position.set(centerX, centerY)
        }).also { it.userData = this }

        body = b

        // Create a rectangular sensor fixture that covers the water area
        PolygonShape().also { shape ->
            shape.setAsBox(width / 2f, height / 2f)
            b.createFixture(FixtureDef().apply {
                this.shape = shape
                isSensor = true
            })
            // Important: dispose Box2D shape after the fixture has been created
            shape.dispose()
        }

        // Initialize wave columns (including both edges)
        if (waves) {
            val startX = centerX - width / 2f
            val bottomY = centerY - height / 2f
            val topY = centerY + height / 2f

            // Number of segments across the width (ensure at least 1)
            val segments = max(1, (width / columnSeparation).toInt())

            repeat(segments + 1) { i ->
                val cx = startX + i * columnSeparation
                columns.add(WaterColumn(cx, bottomY, topY, topY, 0f))
            }
        }
    }

    /**
     * Updates the position of bodies in contact with water. To do this, it applies a force that counteracts
     * gravity by calculating the area in contact, centroid and force required.
     */
    fun update() {
        // Guard: if there is no water body, nothing to update
        val waterBody = body ?: return
        val world = waterBody.world

        // Iterate over all current contact pairs between water (fixtureA) and an object (fixtureB)
        for ((fluidFix, objectFix) in fixturePairs) {
            val clipped: MutableList<Vector2> = ArrayList()

            // Compute intersection polygon (object ∩ fluid); skip if there is no overlap
            if (!IntersectionUtils.findIntersectionOfFixtures(fluidFix, objectFix, clipped)) continue

            // --- Buoyancy (Archimedes) ------------------------------------------------------------
            // Build a Polygon from the clipped points to obtain area and centroid
            val intersectionPoly = IntersectionUtils.getIntersectionPolygon(clipped)

            val centroid = Vector2()
            GeometryUtils.polygonCentroid(
                intersectionPoly.vertices,           // float[] of x,y, x,y, ...
                0,
                intersectionPoly.vertices.size,
                centroid
            )
            val area = intersectionPoly.area()
            val displacedMass = density * area

            // Buoyancy force opposes gravity (applied to the object's body at the centroid)
            val objectBody = objectFix.body
            val fluidBody = fluidFix.body

            val gravity = world.gravity                                 // Vector2(gx, gy)
            val buoyancyForce = Vector2(-gravity.x, -gravity.y).scl(displacedMass)
            objectBody.applyForce(buoyancyForce, centroid, true)

            // --- Hydrodynamics: drag & lift along each polygon edge -------------------------------
            // We need the closed ring of edges, so we iterate indices and wrap the last-to-first edge.
            val n = clipped.size
            for (i in 0 until n) {
                // Edge endpoints and mid point
                val p1 = clipped[i]
                val p2 = clipped[(i + 1) % n]
                val mid = p1.cpy().add(p2).scl(0.5f)

                // Relative velocity at the mid point (object vs. fluid)
                val relVel = objectBody.getLinearVelocityFromWorldPoint(mid)
                    .sub(fluidBody.getLinearVelocityFromWorldPoint(mid))
                val speed = relVel.len()
                // If speed is ~0, no hydrodynamic forces to apply
                if (speed == 0f) continue
                val vHat = relVel.nor()  // unit velocity direction

                // Edge direction and its outward normal (right-handed)
                val edge = p2.cpy().sub(p1)
                val edgeLen = edge.len()
                if (edgeLen == 0f) continue
                val eHat = edge.nor()
                val nHat = Vector2(eHat.y, -eHat.x) // rotate -90°

                // Drag acts opposite to motion and only on leading edges (normal · v ≥ 0)
                val dragDot = nHat.dot(vHat)
                if (dragDot < 0f) continue

                // Common product used by both drag and lift
                val common = edgeLen * density * speed * speed

                // Drag magnitude (clamped)
                val dragMag = min(dragDot * DRAG_MOD * common, MAX_DRAG)
                val dragForce = vHat.cpy().scl(-dragMag)
                objectBody.applyForce(dragForce, mid, true)

                // Lift magnitude (clamped): proportional to alignment of velocity with edge
                val liftDot = eHat.dot(vHat)
                val liftMag = min(dragDot * liftDot * LIFT_MOD * common, MAX_LIFT)
                val liftDir = Vector2(-vHat.y, vHat.x)  // v rotated +90°
                val liftForce = liftDir.scl(liftMag)
                objectBody.applyForce(liftForce, mid, true)

                // Gentle angular damping to reduce spinning when interacting with fluid
                objectBody.applyTorque(-objectBody.angularVelocity / TORQUE_DAMPING, true)
            }

            // --- Wave coupling & splashes ---------------------------------------------------------
            if (waves && area > MIN_SPLASH_AREA && clipped.isNotEmpty()) {
                updateColumns(objectBody, clipped)
            }
        }

        // --- Particle update ----------------------------------------------------------------------
        if (waves && splashParticles && particles.isNotEmpty()) {
            updateParticles()
        }
    }

    /**
     * Update the position of each particle
     */
    private fun updateParticles() {
        val baseY = columns.firstOrNull()?.targetHeight ?: return
        val it = particles.iterator()
        while (it.hasNext()) {
            val particle = it.next()
            val elapsedTime = particle.time + Gdx.graphics.deltaTime

            val y = (baseY + (abs(particle.velocity.y) * elapsedTime)
                    + 0.5f * GRAVITY.y * elapsedTime * elapsedTime)

            if (y < baseY) {
                it.remove()
            } else {
                val x = (particle.initX + particle.velocity.x * elapsedTime)
                particle.time = elapsedTime
                particle.position = Vector2(x, y)
            }
        }
    }

    /**
     * Update the speed of each column in case that a body has touched it.
     *
     * @param body               Body to evaluate
     * @param intersectionPoints Part of the body that is in contact with water
     */
    private fun updateColumns(body: Body, intersectionPoints: MutableList<Vector2>) {
        val minX = intersectionPoints.minOf { it.x }
        val maxX = intersectionPoints.maxOf { it.x }

        columns.forEach { column ->
            if (column.x in minX..maxX) {
                // column points
                val col1 = Vector2(column.x, column.height)
                val col2 = Vector2(column.x, column.y)

                intersectionPoints.zipWithNext().forEach { (p1, p2) ->
                    val intersection = IntersectionUtils.intersection(col1, col2, p1, p2)
                    if (intersection != null && intersection.y < column.height) {
                        if (body.linearVelocity.y < 0 && column.actualBody == null) {
                            column.actualBody = body
                            column.speed = body.linearVelocity.y * 3f / 100f
                            if (splashParticles) createSplashParticles(column)
                        }
                    }
                }
            } else if (body === column.actualBody) {
                column.actualBody = null
            }

            val bodyBelow = body.position.y < column.y
            val actualBelow = column.actualBody?.position?.y?.let { it < column.y } ?: false
            if (bodyBelow || actualBelow) column.actualBody = null
        }
    }

    /**
     * Update the position of each column with respect to the speed that has been applied
     */
    private fun updateWaves() {
        columns.forEach { it.update(dampening, tension) }

        val lDeltas = FloatArray(columns.size)
        val rDeltas = FloatArray(columns.size)
        val s = spread

        repeat(8) {
            columns.zipWithNext().forEachIndexed { idx, (left, right) ->
                val d = s * (right.height - left.height)
                lDeltas[idx + 1] = d
                rDeltas[idx] = -d
                left.speed += d
                right.speed -= d
            }

            columns.zipWithNext().forEachIndexed { idx, (left, right) ->
                left.height += lDeltas[idx + 1]
                right.height += rDeltas[idx]
            }
        }
    }

    /**
     * Create a new splash particle in the given position with a specific velocity
     *
     * @param pos      Init position of the splash particle
     * @param velocity Init velocity of the splash particle
     */
    private fun createParticle(pos: Vector2, velocity: Vector2, radius: Float) {
        val particle = Particle(pos, velocity, radius)
        particles.add(particle)
    }

    /**
     * Creates particles in random position and velocity near to the body
     *
     * @param column We use it to know the speed of the body that is touching it
     */
    private fun createSplashParticles(column: WaterColumn) {
        // Get the body that touched this column; if none, no particles are created
        val body = column.actualBody ?: return

        val y = column.height
        val bodyVel = body.linearVelocity.y.absoluteValue

        // Ignore very slow movements (no splash effect)
        if (bodyVel <= 3f) return

        // Number of particles depends on velocity (at least 1)
        val count = (bodyVel / 8f).toInt().coerceAtLeast(1)

        repeat(count) {
            // Particle start position near the water column, offset with some randomness
            val pos = Vector2(column.x, y).add(getRandomVector(column.targetHeight))

            // Particle velocity:
            // 25% chance: straight up (splash shooting upwards)
            // Otherwise: tilted left or right depending on which side of the body the particle spawns
            val vel = when {
                rand.nextInt(4) == 0 -> {
                    val vy = bodyVel * (0.5f + rand.nextFloat() * 0.5f)   // 50%..100% of bodyVel
                    Vector2(0f, vy)
                }
                else -> {
                    val dir = if (pos.x < body.position.x) -1f else 1f    // Left or right direction
                    val vx = dir * (bodyVel * (0.2f + rand.nextFloat() * 0.2f)) // 20%..40% of bodyVel
                    val vy = bodyVel * (0.3333f + rand.nextFloat() * 0.3333f)   // 33%..66% of bodyVel
                    Vector2(vx, vy)
                }
            }

            // Particle radius between 0.025 and 0.05
            val radius = 0.025f + rand.nextFloat() * (0.05f - 0.025f)

            // Add the new splash particle
            createParticle(pos, vel, radius)
        }
    }

    /**
     * Draws the waves and splash particles if they exist
     *
     * @param camera Camera used in the current stage
     */
    fun draw(camera: Camera) {
        // Make sure GPU-dependent resources exist (safe to call multiple times)
        ensureGraphicsInitialized()

        // Early-out if waves are disabled
        if (!hasWaves()) return

        // Run the wave simulation step before rendering
        updateWaves()

        // Cache local references to avoid repetitive !! and property lookups
        val poly = polyBatch ?: return            // required for water polygons
        val shape = shapeBatch ?: return          // used for debug lines (and rectangles)
        val region = textureWater                 // can be null in debug mode
        val debug = isDebugMode

        // Set projection matrices for this camera
        poly.projectionMatrix = camera.combined
        shape.projectionMatrix = camera.combined

        // --- Draw water surface as quads triangulated into polygons ---
        poly.begin()
        columns.zipWithNext().forEach { (c1, c2) ->
            if (!debug) {
                // Build a vertical quad between two neighboring columns:
                // bottom-left (c1.x, c1.y), top-left (c1.x, c1.height),
                // top-right (c2.x, c2.height), bottom-right (c2.x, c2.y)
                val vertices = floatArrayOf(
                    c1.x, c1.y,
                    c1.x, c1.height,
                    c2.x, c2.height,
                    c2.x, c2.y
                )

                // Guard: texture region must exist when not in debug mode
                val tex = region ?: return@forEach

                // Triangulate the quad and draw it as a PolygonSprite
                val triangles = EarClippingTriangulator().computeTriangles(vertices).toArray()
                val sprite = PolygonSprite(PolygonRegion(tex, vertices, triangles))

                // Slight transparency modulation based on column height (0.95..1.0)
                val alpha = min(1f, max(0.95f, c1.height / c1.targetHeight))

                sprite.draw(poly, alpha)
            } else {
                // Debug mode: just draw the left column as a vertical line
                shape.begin(ShapeType.Line)
                shape.line(Vector2(c1.x, c1.y), Vector2(c1.x, c1.height))
                shape.end()
            }
        }
        poly.end()

        // --- Draw splash particles (if enabled) ---
        if (!hasSplashParticles()) return

        if (!debug) {
            val sb = spriteBatch ?: return
            sb.projectionMatrix = camera.combined
            sb.begin()
            for (p in particles) {
                // Draw each particle as a textured quad (size = diameter)
                sb.draw(textureDrop, p.position.x, p.position.y, p.radius * 2, p.radius * 2)
            }
            sb.end()
        } else {
            // Debug: render particles as wireframe rectangles
            shape.begin(ShapeType.Line)
            for (p in particles) {
                shape.rect(p.position.x, p.position.y, p.radius * 2, p.radius * 2)
            }
            shape.end()
        }
    }

    override fun dispose() {
        if (spriteBatch != null) spriteBatch!!.dispose()
        if (polyBatch != null) polyBatch!!.dispose()
        if (shapeBatch != null) shapeBatch!!.dispose()
        if (textureDrop != null) textureDrop!!.dispose()
        if (textureWater != null) textureWater!!.texture.dispose()
        columns.clear()
        particles.clear()
        fixturePairs.clear()
        body?.let { b ->
            b.world.destroyBody(b)
            body = null
        }
    }

    fun hasWaves(): Boolean {
        return waves
    }

    fun hasSplashParticles(): Boolean {
        return splashParticles
    }

    companion object {
        private val rand = Random.Default
    }
}
