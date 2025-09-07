package com.github.bennyOe.water

import com.badlogic.gdx.math.Polygon
import com.badlogic.gdx.math.Vector2
import com.badlogic.gdx.physics.box2d.Fixture
import com.badlogic.gdx.physics.box2d.PolygonShape
import com.badlogic.gdx.physics.box2d.Shape
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

object IntersectionUtils {
    /**
     * Returns the point where two (infinite) lines intersect, or null if they are parallel/overlapping.
     * @param cp1 Polygon side point 1
     * @param cp2 Polygon side point 2
     * @param s Line start point
     * @param e Line end point
     * @return The point where the two lines intersect or null if they don't cross
     */
    fun intersection(cp1: Vector2, cp2: Vector2, s: Vector2, e: Vector2): Vector2? {
        val dc = Vector2(cp1.x - cp2.x, cp1.y - cp2.y)
        val dp = Vector2(s.x - e.x, s.y - e.y)
        val n1 = cp1.x * cp2.y - cp1.y * cp2.x
        val n2 = s.x * e.y - s.y * e.x
        val denominator = (dc.x * dp.y - dc.y * dp.x)
        if (denominator == 0f) return null // lines are parallel or overlapping
        val inv = 1f / denominator
        return Vector2((n1 * dp.x - n2 * dc.x) * inv, (n1 * dp.y - n2 * dc.y) * inv)
    }

    /**
     * Checks if point p lies to the left of the directed edge (cp1 -> cp2). Used by Sutherland–Hodgman.
     * @param cp1 Polygon point 1
     * @param cp2 Polygon point 2
     * @param p Point to check
     * @return True if the point is inside the polygon
     */
    fun inside(cp1: Vector2, cp2: Vector2, p: Vector2): Boolean {
        return (cp2.x - cp1.x) * (p.y - cp1.y) > (cp2.y - cp1.y) * (p.x - cp1.x)
    }


    /**
     * Finds the points where two fixtures intersects
     * @param fA Fixture A (water)
     * @param fB Fixture B (dynamic body)
     * @param outputVertices It will be set with the points that form the result intersection polygon
     * @return True if the two fixtures intersect
     */
    fun findIntersectionOfFixtures(
        fA: Fixture,
        fB: Fixture,
        outputVertices: MutableList<Vector2>
    ): Boolean {
        // Support only Polygon or Circle fixtures. Early-out otherwise.
        val polyA: PolygonShape = when (fA.shape.type) {
            Shape.Type.Polygon -> fA.shape as PolygonShape
            Shape.Type.Circle  -> circleToSquare(fA)  // approximated by an axis-aligned square
            else               -> return false
        }
        val polyB: PolygonShape = when (fB.shape.type) {
            Shape.Type.Polygon -> fB.shape as PolygonShape
            Shape.Type.Circle  -> circleToSquare(fB)
            else               -> return false
        }

        // We "set" the output list (as documented), so clear it first.
        outputVertices.clear()

        // Build subject polygon from fixture A in world space
        val tmp = Vector2()
        repeat(polyA.vertexCount) { i ->
            polyA.getVertex(i, tmp)                         // local coords into tmp
            val world = fA.body.getWorldPoint(tmp)          // world coords (reuses tmp)
            outputVertices.add(world.cpy())                 // store a copy; avoid aliasing
        }

        // Build clip polygon from fixture B in world space
        val clipPolygon = ArrayList<Vector2>(polyB.vertexCount)
        repeat(polyB.vertexCount) { i ->
            polyB.getVertex(i, tmp)
            val world = fB.body.getWorldPoint(tmp)
            clipPolygon.add(world.cpy())
        }

        // Sutherland–Hodgman clipping: clip subject by each edge of the clip polygon
        var cp1 = clipPolygon.last()
        for (cp2 in clipPolygon) {
            if (outputVertices.isEmpty()) return false

            val input = ArrayList(outputVertices) // snapshot current subject
            outputVertices.clear()

            var s = input.last() // start from the last vertex (closing the loop)
            for (e in input) {
                val eInside = inside(cp1, cp2, e)
                val sInside = inside(cp1, cp2, s)

                if (eInside) {
                    if (!sInside) {
                        // Segment enters the clip half-space: add intersection first
                        intersection(cp1, cp2, s, e)?.let(outputVertices::add)
                    }
                    // Current point is inside: keep it
                    outputVertices.add(e)
                } else if (sInside) {
                    // Segment exits the clip half-space: add intersection only
                    intersection(cp1, cp2, s, e)?.let(outputVertices::add)
                }
                s = e
            }
            cp1 = cp2
        }

        return outputVertices.isNotEmpty()
    }

    /**
     * Builds a libGDX Polygon from a list of vertices (x0,y0,x1,y1,...).
     * @param vertices Vertices of the polygon
     * @return Polygon result
     */
    fun getIntersectionPolygon(vertices: List<Vector2>): Polygon {
        // Guard: no vertices → empty polygon
        if (vertices.isEmpty()) return Polygon(floatArrayOf())

        // Pack (x,y) pairs into a flat float array
        val points = FloatArray(vertices.size * 2)
        var k = 0
        for (v in vertices) {
            points[k++] = v.x
            points[k++] = v.y
        }

        return Polygon(points)
    }

    /**
     * Approximates a circle as an axis-aligned square (needed because clipping works on vertices).
     * @param fixture Circle fixture
     * @return A square instead of the circle
     */
    private fun circleToSquare(fixture: Fixture): PolygonShape {
        val position = fixture.body.localCenter
        val r = fixture.shape.radius
        return PolygonShape().apply { setAsBox(r, r, position, 0f) }
    }

    /**
     * Obtains a random vector
     * @param maxLength Max length
     * @return A random vector
     */
    fun getRandomVector(maxLength: Float): Vector2 {
        val angle = Random.nextFloat() * (2f * PI.toFloat()) - PI.toFloat()
        val radius = sqrt(Random.nextFloat()) * maxLength // area-uniform
        return fromPolar(angle, radius)
    }

    /**
     * Constructs a Vector2 from polar coordinates (angle in radians, magnitude).
     */
    private fun fromPolar(angle: Float, magnitude: Float): Vector2 {
        val c = cos(angle)
        val s = sin(angle)
        return Vector2((c * magnitude), (s * magnitude))
    }
}
