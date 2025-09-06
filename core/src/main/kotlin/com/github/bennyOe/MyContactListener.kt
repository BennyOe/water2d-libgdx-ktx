package com.github.bennyOe

import com.badlogic.gdx.physics.box2d.BodyDef.BodyType.DynamicBody
import com.badlogic.gdx.physics.box2d.Contact
import com.badlogic.gdx.physics.box2d.ContactImpulse
import com.badlogic.gdx.physics.box2d.ContactListener
import com.badlogic.gdx.physics.box2d.Fixture
import com.badlogic.gdx.physics.box2d.Manifold
import com.github.bennyOe.water.Water

class MyContactListener : ContactListener {

    override fun beginContact(contact: Contact) {
        val match = contact.findWaterAndDynamicFixture() ?: return
        val (water, dynamicFixture, waterFixture) = match
        // Store the pair in a consistent orientation: (waterFixture, dynamicFixture)
        water.fixturePairs.add(waterFixture to dynamicFixture)
    }

    override fun endContact(contact: Contact) {
        val match = contact.findWaterAndDynamicFixture() ?: return
        val (water, dynamicFixture, waterFixture) = match
        // Remove the exact same pair we added in beginContact
        water.fixturePairs.remove(waterFixture to dynamicFixture)
    }

    override fun preSolve(contact: Contact, oldManifold: Manifold) = Unit
    override fun postSolve(contact: Contact, impulse: ContactImpulse) = Unit
}

/**
 * Returns a triple of (waterInstance, dynamicFixture, waterFixture) if one side is Water and the
 * other side is a DynamicBody; otherwise null. Works regardless of which side is which.
 */
private fun Contact.findWaterAndDynamicFixture(): Triple<Water, Fixture, Fixture>? {
    val a = fixtureA
    val b = fixtureB

    val waterA = (a.body.userData as? Water)
    val waterB = (b.body.userData as? Water)

    return when {
        waterA != null && b.body.type == DynamicBody -> Triple(waterA, b, a)
        waterB != null && a.body.type == DynamicBody -> Triple(waterB, a, b)
        else -> null
    }
}
