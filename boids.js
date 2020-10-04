import { Pool, setLength, vec2 } from './deps.js'
import setAngle  from './vec2-set-angle.js'


// much of this was inspired by code from Ian
// https://github.com/ianmcgregor/boid/blob/master/src/boid.js

// TODO: opensteer implementation from Craig has better algorithms for many of
// these steering behaviors:      http://www.red3d.com/cwr/steer/gdc99/

function createSteeringComponent (options={}) {
    return {
        // common vehicle steering parameters
        steeringForce: vec2.create(),  // MUTABLE
        maxForce: options.maxForce || 60,

        // collision avoidance
        MAX_SEE_AHEAD: options.maxSeeAhead || 100,

        // flock
        maxDistance: 300,
        minDistance: 60,

        // wander
        wanderDistance: 10,
        wanderRadius: 5,
        wanderAngle: 0,      // MUTABLE
        wanderRange: 1,

        // arrive
        arriveThreshold: 32,

        // leader follow
        leaderBehindDistance: 50,
        leaderSightRadius: 120,

        // queueing
        maxQueueAhead: 40,
        maxQueueRadius: 20,
    }
}


/*
  boid entity structure:
      steering:
          ... boid related settings
      aabb
          position
          width
          height
      rigidBody
         maxVelocity
         velocity
*/


// look at velocity of target boid and try to predict where it's going with the aim of catching it
function steerForPursuit (pursuerBoid, targetPosition, targetVelocity) {
    const maxSpeedSq = pursuerBoid.rigidBody.maxVelocity * pursuerBoid.rigidBody.maxVelocity
    const lookAheadTime = vec2.squaredDistance(pursuerBoid.aabb.position, targetPosition) / maxSpeedSq

    const scaledVelocity = Pool.malloc()
    vec2.scale(scaledVelocity, targetVelocity, lookAheadTime)

    const predictedTarget = Pool.malloc()

    vec2.add(predictedTarget, targetPosition, scaledVelocity)

    steerForSeek(pursuerBoid, predictedTarget)

    //steerForArrival(pursuerBoid, predictedTarget)

    Pool.free(scaledVelocity)
    Pool.free(predictedTarget)
}


// seek until within arriveThreshold
function steerForArrival (boid, targetVec) {
    const desiredVelocity = vec2.subtract(Pool.malloc(), targetVec, boid.aabb.position)
    vec2.normalize(desiredVelocity, desiredVelocity)

    const arriveThresholdSq = boid.steering.arriveThreshold * boid.steering.arriveThreshold
    const distanceSq = vec2.squaredDistance(boid.aabb.position, targetVec)
    if (distanceSq > arriveThresholdSq) {
        vec2.scale(desiredVelocity, desiredVelocity, boid.rigidBody.maxVelocity)
    } else {
        const scalar = boid.rigidBody.maxVelocity * distanceSq / arriveThresholdSq
        vec2.scale(desiredVelocity, desiredVelocity, scalar)
    }

    const force = vec2.subtract(Pool.malloc(), desiredVelocity, boid.rigidBody.velocity)

    vec2.add(boid.steering.steeringForce, boid.steering.steeringForce, force)
    Pool.free(force)
}


// steer torwards targetVec
function steerForSeek (boid, targetVec) {
    const desiredVelocity = vec2.subtract(Pool.malloc(), targetVec, boid.aabb.position)
    vec2.normalize(desiredVelocity, desiredVelocity)

    vec2.scale(desiredVelocity, desiredVelocity, boid.rigidBody.maxVelocity)

    const force = Pool.malloc()
    vec2.subtract(force, desiredVelocity, boid.rigidBody.velocity)
    vec2.add(boid.steering.steeringForce, boid.steering.steeringForce, force)

    Pool.free(force)
}


// wander around, changing angle by a limited amount each tick
function steerForWander (boid) {
    const center = Pool.malloc()
    vec2.normalize(center, boid.rigidBody.velocity)
    vec2.scale(center, center, boid.steering.wanderDistance)

    const offset = Pool.malloc()
    setLength(offset, boid.steering.wanderRadius)
    setAngle(offset, boid.steering.wanderAngle)
    boid.steering.wanderAngle += Math.random() * boid.steering.wanderRange - boid.steering.wanderRange * 0.5

    vec2.add(center, center, offset)
    vec2.add(boid.steering.steeringForce, boid.steering.steeringForce, center)

    Pool.free(offset)
    Pool.free(center)
}


// look at velocity of boid and try to predict where it's going
// @param Object targetBoid the boid to evade
function steerForEvasion (evadingBoid, targetBoid) {
    const maxSpeedSq = evadingBoid.rigidBody.maxVelocity * evadingBoid.rigidBody.maxVelocity
    const lookAheadTime = vec2.squaredDistance(evadingBoid.aabb.position, targetBoid.aabb.position) / maxSpeedSq

    const scaledVelocity = Pool.malloc()
    vec2.scale(scaledVelocity, targetBoid.rigidBody.velocity, lookAheadTime)

    const predictedTarget = Pool.malloc()
    vec2.scale(predictedTarget, targetBoid.aabb.position, scaledVelocity)

    steerFoFlee(evadingBoid, predictedTarget)

    Pool.free(scaledVelocity)
    Pool.free(predictedTarget)
}


// steer away from targetVec
function steerForFlee (fleeingBoid, targetVec) {
    const desiredVelocity = Pool.malloc()
    vec2.subtract(desiredVelocity, targetVec, fleeingBoid.aabb.position)
    vec2.normalize(desiredVelocity, desiredVelocity)
    vec2.scale(desiredVelocity, desiredVelocity, fleeingBoid.rigidBody.maxVelocity)

    const force = Pool.malloc()
    vec2.subtract(force, desiredVelocity, fleeingBoid.rigidBody.velocity)
    vec2.subtract(fleeingBoid.steering.steeringForce, fleeingBoid.steering.steeringForce, force)
    Pool.free(force)
}


// group of boids loosely move together
function steerForFlock (boid, boids) {
    const averageVelocity = Pool.malloc(boid.rigidBody.velocity[0], boid.rigidBody.velocity[1])
    const averagePosition = Pool.malloc()
    let inSightCount = 0

    boids.forEach(function (b) {
        if ((b !== boid) && _inSight(boid, b)) {
            vec2.add(averageVelocity, averageVelocity, b.rigidBody.velocity)
            vec2.add(averagePosition, averagePosition, b.aabb.position)

            const tooClose = vec2.distance(boid.aabb.position, b.aabb.position) < boid.steering.minDistance
            if (tooClose)
                steerForFlee(boid, b.aabb.position)
            inSightCount++
        }
    })

    if (inSightCount > 0) {
        vec2.scale(averageVelocity, averageVelocity, 1/inSightCount)
        vec2.scale(averagePosition, averagePosition, 1/inSightCount)

        steerForSeek(boid, averagePosition)

        const delta = Pool.malloc()
        vec2.subtract(delta, averageVelocity, boid.rigidBody.velocity)
        vec2.add(boid.steering.steeringForce, boid.steering.steeringForce, delta)
        Pool.free(delta)
    }

    Pool.free(averageVelocity)
    Pool.free(averagePosition)
}


// http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-leader-following--gamedev-10810
function steerForFollowLeader (boid, leader, boids) {
    // Calculate the ahead point
    const tv = vec2.normalize(Pool.malloc(), leader.rigidBody.velocity)
    vec2.scale(tv, tv, boid.steering.leaderBehindDistance)

    const ahead = vec2.add(Pool.malloc(), leader.aabb.position, tv)

    // Calculate the behind point
    vec2.negate(tv, tv)
    const behind = vec2.add(Pool.malloc(), leader.aabb.position, tv)

    // If the character is on the leader's sight, add a force
    // to evade the route immediately.
    const maxDistSq = boid.steering.leaderSightRadius * boid.steering.leaderSightRadius
    const isOnLeaderSight = vec2.squaredDistance(boid.aabb.position, ahead) <= maxDistSq || vec2.squaredDistance(leader.aabb.position, boid.aabb.position) <= maxDistSq

    if (isOnLeaderSight)
        steerForEvasion(boid, leader)

    // Creates a force to arrive at the behind point
    steerForArrival(boid, behind)

    // Add separation force
    steerForFlock(boid, boids)

    Pool.free(tv)
    Pool.free(behind)
}


// http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-queue--gamedev-14365
function steerForQueueing (boid, boids) {
    const neighbor = _getNeighborAhead(boid, boids)
    if (!neighbor)
        return

    const brake = Pool.malloc()
    vec2.negate(brake, boid.steering.steeringForce)
    vec2.scale(brake, brake, 0.8)

    const v = Pool.malloc()
    vec2.negate(v, boid.rigidBody.velocity)

    vec2.add(brake, brake, v)
    vec2.add(brake, brake, steerForFlock(boid, boids))

    if (vec2.distance(object.aabb.position, neighbor.aabb.position) <= boid.steering.maxQueueRadius)
      vec2.scale(boid.rigidBody.velocity, boid.rigidBody.velocity, 0.3)

    vec2.add(boid.steering.steeringForce, boid.steeering.steeringForce, brake)
    Pool.free(v)
    Pool.free(brake)
}


// http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
// @param Map boids
function steerForCollisionAvoidance (boid, boids) {
    const ahead = vec2.normalize(Pool.malloc(), boid.rigidBody.velocity)
    vec2.scaleAndAdd(ahead, boid.aabb.position, ahead, boid.steering.MAX_SEE_AHEAD)

    const ahead2 = vec2.normalize(Pool.malloc(), boid.rigidBody.velocity)
    vec2.scaleAndAdd(ahead2, boid.aabb.position, ahead2, boid.steering.MAX_SEE_AHEAD * 0.5)

    const mostThreatening = _findMostThreateningObstacle(boid, boids, ahead, ahead2)

    Pool.free(ahead2)

    if (mostThreatening) {
        // obstacle, avoidance force needed
        const avoidance = vec2.subtract(Pool.malloc(), ahead, mostThreatening.aabb.position)
        vec2.normalize(avoidance, avoidance)
        vec2.scaleAndAdd(boid.steering.steeringForce, boid.steering.steeringForce, avoidance, boid.steering.maxForce)
        Pool.free(avoidance)
    }
    Pool.free(ahead)
}


// @param Map objects
// @param vec2 ahead
// @param vec2 ahead2
function _findMostThreateningObstacle (boid, boids, ahead, ahead2) {
    let mostThreatening

    for (const next of boids) {
        if (next === boid)
            return
        const radius = next.radius || Math.max(next.aabb.width, next.aabb.height)
        const collision = _lineIntersectsCircle(ahead, ahead2, next.aabb.position, radius * 1.1)
        // "position" is the character's current position
        if (collision && (!mostThreatening || vec2.distance(boid.aabb.position, next.aabb.position) < vec2.distance(boid.aabb.position, mostThreatening.aabb.position)))
            mostThreatening = next
    }

    return mostThreatening
}


// get the first neighbor that is ahead of this entity
function _getNeighborAhead (boid, neighbors) {
    const qa = vec2.normalize(Pool.malloc(), boid.rigidBody.velocity)
    vec2.scale(qa, qa, boid.steering.maxQueueAhead)

    const ahead = vec2.add(Pool.malloc(), boid.aabb.position, qa)

    const result = neighbors.find((neighbor) => {
        const d = vec2.distance(ahead, neighbor.aabb.position)
        if ((neighbour !== boid) && (d <= boid.steering.maxQueueRadius))
            return neighbor
    })

    Pool.free(ahead)
    Pool.free(qa)
    return result
}


// is boid close enough to be in sight and facing
function _inSight (lookingBoid, boid) {
    if (vec2.distance(lookingBoid.aabb.position, boid.aabb.position) > lookingBoid.steering.maxDistance)
        return false

    const heading = vec2.normalize(Pool.malloc(), lookingBoid.rigidBody.velocity)

    const difference = vec2.subtract(Pool.malloc(), boid.aabb.position, lookingBoid.aabb.position)

    const dotProd = vec2.dot(difference, heading)
    Pool.free(heading)
    Pool.free(difference)

    return dotProd >= 0
}


function _lineIntersectsCircle (ahead, ahead2, center, radius) {
    // the property "center" of the obstacle is a vec2
    return vec2.distance(center, ahead) <= radius || vec2.distance(center, ahead2) <= radius
}


export default {
    createSteeringComponent,
    steerForArrival, steerForEvasion, steerForFlee, steerForFlock, steerForFollowLeader,
    steerForPursuit, steerForQueueing, steerForSeek, steerForWander, steerForCollisionAvoidance
}
