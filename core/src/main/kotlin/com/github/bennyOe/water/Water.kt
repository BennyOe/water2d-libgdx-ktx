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
import com.github.bennyOe.water.IntersectionUtils.getRandomVector
import java.util.LinkedHashSet

import kotlin.math.abs
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

    val columnSeparation: Float = 0.04f // 4 px between every column

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
     * @param x      Position of the x coordinate of the center of the body
     * @param y      Position of the y coordinate of the center of the body
     * @param width  Body width
     * @param height Body height
     */
    fun createBody(world: World, x: Float, y: Float, width: Float, height: Float) {
        val bodyDef = BodyDef()
        bodyDef.type = BodyType.StaticBody
        bodyDef.position.set(x, y)

        // Create our body in the world using our body definition
        body = world.createBody(bodyDef)
        body!!.userData = this

        val square = PolygonShape()
        square.setAsBox(width / 2, height / 2)

        // Create a fixture definition to apply our shape to
        val fixtureDef = FixtureDef()
        fixtureDef.shape = square

        // Must be a sensor
        fixtureDef.isSensor = true

        // Create our fixture and attach it to the body
        body!!.createFixture(fixtureDef)

        square.dispose()

        // Water columns (waves)
        if (waves) {
            val count = (width / columnSeparation).toInt()
            for (i in 0..count) {
                val cx = i * columnSeparation + x - width / 2
                columns.add(WaterColumn(cx, y - height / 2, y + height / 2, y + height / 2, 0f))
            }
        }
    }

    /**
     * Updates the position of bodies in contact with water. To do this, it applies a force that counteracts
     * gravity by calculating the area in contact, centroid and force required.
     */
    fun update() {
        if (body != null) {
            val world = body!!.world
            for (pair in fixturePairs) {
                val fixtureA: Fixture = pair.first
                val fixtureB: Fixture = pair.second

                val clippedPolygon: MutableList<Vector2> = ArrayList()
                if (IntersectionUtils.findIntersectionOfFixtures(fixtureA, fixtureB, clippedPolygon)) {
                    // find centroid and area

                    val interPolygon = IntersectionUtils.getIntersectionPolygon(clippedPolygon)
                    val centroid = Vector2()
                    GeometryUtils.polygonCentroid(interPolygon.vertices, 0, interPolygon.vertices.size, centroid)
                    val area = interPolygon.area()

                    /* Get fixtures bodies */
                    val fluidBody = fixtureA.body
                    val fixtureBody = fixtureB.body

                    // apply buoyancy force (fixtureA is the fluid)
                    val displacedMass = this.density * area
                    val buoyancyForce = Vector2(
                        displacedMass * -world.getGravity().x,
                        displacedMass * -world.getGravity().y
                    )
                    fixtureB.body.applyForce(buoyancyForce, centroid, true)


                    /* Apply drag and lift forces */
                    val polygonVertices = clippedPolygon.size
                    for (i in 0..<polygonVertices) {
                        /* End points and mid-point of the edge */

                        val firstPoint = clippedPolygon[i]
                        val secondPoint = clippedPolygon[(i + 1) % polygonVertices]
                        val midPoint = firstPoint.cpy().add(secondPoint).scl(0.5f)

                        /*
                         * Find relative velocity between the object and the fluid at edge
                         * mid-point.
                         */
                        val velocityDirection = Vector2(
                            fixtureBody
                                .getLinearVelocityFromWorldPoint(midPoint)
                                .sub(fluidBody.getLinearVelocityFromWorldPoint(midPoint))
                        )
                        val velocity = velocityDirection.len()
                        velocityDirection.nor()

                        val edge = secondPoint.cpy().sub(firstPoint)
                        val edgeLength = edge.len()
                        edge.nor()

                        val normal = Vector2(edge.y, -edge.x)
                        val dragDot = normal.dot(velocityDirection)

                        if (dragDot >= 0) {

                            /*
                             * Normal don't point backwards. This is a leading edge. Store
                             * the result of multiply edgeLength, density and velocity
                             * squared
                             */

                            val tempProduct = edgeLength * density * velocity * velocity

                            var drag = dragDot * DRAG_MOD * tempProduct
                            drag = min(drag, MAX_DRAG)
                            val dragForce = velocityDirection.cpy().scl(-drag)
                            fixtureBody.applyForce(dragForce, midPoint, true)

                            /* Apply lift force */
                            val liftDot = edge.dot(velocityDirection)
                            var lift = dragDot * liftDot * LIFT_MOD * tempProduct
                            lift = min(lift, MAX_LIFT)
                            val liftDirection = Vector2(
                                -velocityDirection.y,
                                velocityDirection.x
                            )
                            val liftForce = liftDirection.scl(lift)
                            fixtureBody.applyForce(liftForce, midPoint, true)


                            fixtureBody.applyTorque(-fixtureBody.angularVelocity / TORQUE_DAMPING, true)
                        }
                    }

                    if (waves && area > MIN_SPLASH_AREA) {
                        if (clippedPolygon.isNotEmpty()) {
                            updateColumns(fixtureB.body, clippedPolygon)
                        }
                    }
                }
            }
        }

        if (waves && splashParticles && !particles.isEmpty()) {
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
                + 0.5f * -10f * elapsedTime * elapsedTime)

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

        for (i in columns.indices) {
            val column = columns[i]

            if (column.x in minX..maxX) {
                // column points
                val col1 = Vector2(column.x, column.height)
                val col2 = Vector2(column.x, column.y)

                for (j in 0 until intersectionPoints.size - 1) {
                    val p1 = intersectionPoints[j]
                    val p2 = intersectionPoints[j + 1]
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
        for (i in columns.indices) {
            columns[i].update(dampening, tension)
        }

        val lDeltas = FloatArray(columns.size)
        val rDeltas = FloatArray(columns.size)

        // do some passes where columns pull on their neighbours
        repeat(8) {
            for (i in columns.indices) {
                if (i > 0) {
                    lDeltas[i] = this.spread * (columns[i].height - columns[i - 1].height)
                    columns[i - 1].speed += lDeltas[i]
                }
                if (i < columns.size - 1) {
                    rDeltas[i] = this.spread * (columns[i].height - columns[i + 1].height)
                    columns[i + 1].speed += rDeltas[i]
                }
            }

            for (i in columns.indices) {
                if (i > 0) columns[i - 1].height += lDeltas[i]
                if (i < columns.size - 1) columns[i + 1].height += rDeltas[i]
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
        val y = column.height
        val bodyVel = abs(column.actualBody!!.getLinearVelocity().y)

        if (abs(bodyVel) > 3f) {
            var i = 0
            while (i < bodyVel / 8) {
                val pos = Vector2(column.x, y).add(getRandomVector(column.targetHeight))
                val vel: Vector2 = if (rand.nextInt(4) == 0) Vector2(0f, bodyVel / 2 + rand.nextFloat() * bodyVel / 2)
                else if (pos.x < column.actualBody!!.getPosition().x) Vector2(
                    -bodyVel / 5 + rand.nextFloat() * bodyVel / 5,
                    bodyVel / 3 + rand.nextFloat() * bodyVel / 3
                )
                else Vector2(
                    bodyVel / 5 + rand.nextFloat() * bodyVel / 5,
                    bodyVel / 3 + rand.nextFloat() * bodyVel / 3
                )

                val radius = rand.nextFloat() * (0.05f - 0.025f) + 0.025f

                this.createParticle(pos, vel, radius)
                i++
            }
        }
    }

    /**
     * Draws the waves and splash particles if they exist
     *
     * @param camera Camera used in the current stage
     */
    fun draw(camera: Camera) {
        // Ensure render resources are initialized during the render phase (after Application is ready)
        ensureGraphicsInitialized()
        if (hasWaves()) {
            updateWaves()

            polyBatch!!.setProjectionMatrix(camera.combined)
            shapeBatch!!.setProjectionMatrix(camera.combined)

            // draw columns water
            polyBatch!!.begin()
            columns.zipWithNext().forEach { (c1, c2) ->
                if (!this.isDebugMode) {
                    val vertices = floatArrayOf(
                        c1.x, c1.y, c1.x, c1.height, c2.x, c2.height, c2.x,
                        c2.y
                    )
                    val sprite = PolygonSprite(
                        PolygonRegion(
                            textureWater, vertices,
                            EarClippingTriangulator().computeTriangles(vertices).toArray()
                        )
                    )
                    sprite.draw(
                        polyBatch,
                        min(1f, max(0.95f, c1.height / c1.targetHeight))
                    ) // remove transparency for waves here if you don't want it
                } else {
                    shapeBatch!!.begin(ShapeType.Line)
                    shapeBatch!!.line(Vector2(c1.x, c1.y), Vector2(c1.x, c1.height))
                    shapeBatch!!.end()
                }
            }
            polyBatch!!.end()

            // draw splash particles
            if (hasSplashParticles()) {
                if (!this.isDebugMode) {
                    spriteBatch!!.setProjectionMatrix(camera.combined)
                    spriteBatch!!.begin()
                    for (p in particles) {
                        spriteBatch!!.draw(textureDrop, p.position.x, p.position.y, p.radius * 2, p.radius * 2)
                    }
                    spriteBatch!!.end()
                } else {
                    shapeBatch!!.setProjectionMatrix(camera.combined)
                    shapeBatch!!.begin(ShapeType.Line)
                    for (p in particles) {
                        shapeBatch!!.rect(p.position.x, p.position.y, p.radius * 2, p.radius * 2)
                    }
                    shapeBatch!!.end()
                }
            }
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
