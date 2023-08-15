import Alea                 from 'https://cdn.skypack.dev/pin/alea@v1.0.0-P9lu4rchYeqab9T0CblM/mode=imports/optimized/alea.js'
import Path                 from './polyline-path.js'
import * as VehicleSteering from './steering-vehicle.js'
import { clamp, vec2, segmentSphereOverlap, perpendicularComponent } from './deps.js'
import inBoidNeighborhood  from './in-boid-neighborhood.js'
import lerp                from 'https://cdn.skypack.dev/pin/lerp@v1.0.3-fjXpN7X7nWMDOQ8ERPDB/mode=imports,min/optimized/lerp.js'
import limitDeviationAngle from './limit-deviation-angle.js'
import vec2SetLength       from 'https://cdn.jsdelivr.net/gh/mreinstein/vec2-gap/set-length.js'
import vec2Truncate        from 'https://cdn.jsdelivr.net/gh/mreinstein/vec2-gap/truncate.js'


const defaultSeed = Math.random()
const defaultRng = new Alea(defaultSeed)

const ORIGIN = vec2.fromValues(0, 0)

// tmp vars to avoid garbage collection
const _clippedForce = vec2.create()
const _newVelocity = vec2.create()
const _desiredVelocity = vec2.create()
const _heading = vec2.create()
const _difference = vec2.create()
const _averageVelocity = vec2.create()
const _averagePosition = vec2.create()
const _deltaVelocity = vec2.create()
const _center = vec2.create()
const _offset = vec2.create()
const _scaledVelocity = vec2.create()
const _predictedTarget = vec2.create()
const _ahead = vec2.create()
const _tv = vec2.create()
const _behind = vec2.create()
const _tmp1 = vec2.create()
const _tmp2 = vec2.create()

/*
  boid entity structure:
      steering:
          ... boid related settings
      aabb
          position
          width
          height
      rigidBody
         maxSpeed
         velocity
*/


export function createSteeringComponent (options={}) {
    return {
        // common vehicle steering parameters
        maxForce: options.maxForce ?? 60,

        // collision avoidance
        MAX_SEE_AHEAD: options.maxSeeAhead ?? 40,

        // flock
        radius: options.radius ?? 10, // radius of sphere encapsulating collision body
        maxDistance: 300,
        minDistance: 60,

        // wander
        wanderDistance: 10,
        wanderRadius: 5,
        wanderAngle: 0,      // MUTABLE
        wanderRange: 1,

        // arrive
        arriveThreshold: options.arrivalThreshold ?? 50,

        // leader follow
        leaderBehindDistance: 32,
        leaderSightRadius: 200,

        // path follow
        pathIndex: 0,

        // queueing
        maxQueueAhead: 40,
        maxQueueRadius: 20,
    }
}


// apply a given steering force to our momentum,
// adjusting our orientation to maintain velocity-alignment.
//
// @param Number elapsedTime  seconds elapsed (dt)
export function applySteeringForce (boid, force/*, elapsedTime*/) {

    // enforce limit on magnitude of steering force
    vec2Truncate(_clippedForce, force, boid.steering.maxForce)

    // Euler integrate (per frame) acceleration into velocity
    const newVelocity = vec2.create()
    vec2.add(_newVelocity, boid.rigidBody.velocity, _clippedForce)

    // enforce speed limit
    vec2Truncate(boid.rigidBody.velocity, _newVelocity, boid.rigidBody.maxSpeed)

    // integrating position is disabled intentionally. In cases where a rigidBody physics system performs collision response,
    // updating position here directly interferes with that.

    // Euler integrate (per frame) velocity into position
    //vec2.scaleAndAdd(boid.transform.position, boid.transform.position, boid.rigidBody.velocity, elapsedTime)
}


// steer torwards target
export function steerForSeek (out, boid, target) {
    vec2.subtract(_desiredVelocity, target, boid.aabb.position)
    vec2SetLength(_desiredVelocity, _desiredVelocity, boid.rigidBody.maxSpeed)

    return vec2.subtract(out, _desiredVelocity, boid.rigidBody.velocity)
}


// seek until within arriveThreshold
export function steerForArrival (out, boid, target) {
    vec2.subtract(_desiredVelocity, target, boid.aabb.position)
    vec2.normalize(_desiredVelocity, _desiredVelocity)

    const distanceSq = vec2.squaredDistance(boid.aabb.position, target)
    const arriveThresholdSq = boid.steering.arriveThreshold * boid.steering.arriveThreshold

    if (distanceSq > arriveThresholdSq) {
        vec2.scale(_desiredVelocity, _desiredVelocity, boid.rigidBody.maxSpeed)
    } else {
        const scalar = boid.rigidBody.maxSpeed * distanceSq / arriveThresholdSq
        vec2.scale(_desiredVelocity, _desiredVelocity, scalar)
    }

    return vec2.subtract(out, _desiredVelocity, boid.rigidBody.velocity)
}


// follow a path made up of an array of vectors
export function steerForFollowPath (out, boid, path, direction, predictionTime, loop=false) {
    const wayPoint = path.points[boid.steering.pathIndex]
    if (!wayPoint) {
        boid.steering.pathIndex = 0
        return
    }

    const pathThresholdSq = path.radius * path.radius
    if (vec2.squaredDistance(boid.transform.position, wayPoint) < pathThresholdSq) {
        if (boid.steering.pathIndex == path.pointCount - 1) {
            if (loop)
                boid.steering.pathIndex = 0
        } else {
            boid.steering.pathIndex++
        }
    }

    if (boid.steering.pathIndex >= path.pointCount - 1 && !loop)
        steerForArrival(out, boid, wayPoint)
    else
        steerForSeek(out, boid, wayPoint)

    return out
}


// steer away from targetVec
export function steerForFlee (out, fleeingBoid, targetVec) {
    vec2.subtract(_desiredVelocity, targetVec, fleeingBoid.aabb.position)
    vec2.normalize(_desiredVelocity, _desiredVelocity)
    vec2.scale(_desiredVelocity, _desiredVelocity, fleeingBoid.rigidBody.maxSpeed)

    vec2.subtract(out, _desiredVelocity, fleeingBoid.rigidBody.velocity)
    return vec2.negate(out, out)
}


// is boid close enough to be in sight and facing
function inSight (lookingBoid, boid) {
    if (vec2.distance(lookingBoid.aabb.position, boid.aabb.position) > lookingBoid.steering.maxDistance)
        return false

    vec2.normalize(_heading, lookingBoid.rigidBody.velocity)

    vec2.subtract(_difference, boid.aabb.position, lookingBoid.aabb.position)

    return vec2.dot(_difference, _heading) >= 0
}


export function steerForFlock (out, boid, flock) {
    vec2.copy(_averageVelocity, boid.rigidBody.velocity)
    vec2.set(_averagePosition, 0, 0)
    let inSightCount = 0

    const minDistanceSq = boid.steering.minDistance * boid.steering.minDistance

    for (let i = 0; i < flock.length; i++) {
        const b = flock[i]
        if (b !== boid && inSight(boid, b)) {

            vec2.add(_averageVelocity, _averageVelocity, b.rigidBody.velocity)
            vec2.add(_averagePosition, _averagePosition, b.transform.position)

            if (vec2.squaredDistance(boid.transform.position, b.transform.position) < minDistanceSq)
                steerForFlee(out, boid, b.transform.position)

            inSightCount++
        }
    }

    if (inSightCount > 0) {
        vec2.scale(_averageVelocity, _averageVelocity, 1/inSightCount)
        vec2.scale(_averagePosition, _averagePosition, 1/inSightCount)
        steerForSeek(out, boid, _averagePosition)
        vec2.subtract(_deltaVelocity, _averageVelocity, boid.rigidBody.velocity)
        vec2.add(out, out, _deltaVelocity)
    } else {
        vec2.set(out, 0, 0)
    }
    
    return out
}


// wander around, changing angle by a limited amount each tick
export function steerForWander (out, boid, random=defaultRng) {
    vec2.normalize(_center, boid.rigidBody.velocity)
    vec2.scale(_center, _center, boid.steering.wanderDistance)

    vec2.set(_offset, boid.steering.wanderRadius, 0)

    vec2.rotate(_offset, _offset, ORIGIN, boid.steering.wanderAngle)

    boid.steering.wanderAngle += random() * boid.steering.wanderRange - boid.steering.wanderRange * 0.5

    return vec2.add(out, _center, _offset)
}


// look at velocity of boid and try to predict where it's going
export function steerForPursuit (out, pursuerBoid, targetPosition, targetVelocity) {

    const maxSpeedSq = pursuerBoid.rigidBody.maxSpeed * pursuerBoid.rigidBody.maxSpeed
    const lookAheadTime = vec2.squaredDistance(pursuerBoid.aabb.position, targetPosition) / maxSpeedSq

    vec2.scale(_scaledVelocity, targetVelocity, lookAheadTime)

    vec2.add(_predictedTarget, targetPosition, _scaledVelocity)

    steerForSeek(out, pursuerBoid, _predictedTarget)

    return out
}


// look at velocity of boid and try to predict where it's going
// @param Object menaceBoid the boid to evade
export function steerForEvasion (out, evadingBoid, menaceBoid) {
    const maxSpeedSq = evadingBoid.rigidBody.maxSpeed * evadingBoid.rigidBody.maxSpeed
    const lookAheadTime = vec2.squaredDistance(evadingBoid.aabb.position, menaceBoid.aabb.position) / maxSpeedSq
    
    vec2.scale(_scaledVelocity, menaceBoid.rigidBody.velocity, lookAheadTime)

    vec2.add(_predictedTarget, menaceBoid.aabb.position, _scaledVelocity)

    steerForFlee(out, evadingBoid, _predictedTarget)

    return out
}


// http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-leader-following--gamedev-10810
export function steerForFollowLeader (out, boid, leader, boids) {
    // Calculate the ahead point
    vec2.normalize(_tv, leader.rigidBody.velocity)
    vec2.scale(_tv, _tv, boid.steering.leaderBehindDistance)

    vec2.add(_ahead, leader.aabb.position, _tv)

    // Calculate the behind point
    vec2.negate(_tv, _tv)
    vec2.add(_behind, leader.aabb.position, _tv)

    // If the character is on the leader's sight, add a force
    // to evade the route immediately.
    const maxDistSq = boid.steering.leaderSightRadius * boid.steering.leaderSightRadius
    const isOnLeaderSight = vec2.squaredDistance(boid.aabb.position, _ahead) <= maxDistSq ||
                            vec2.squaredDistance(leader.aabb.position, boid.aabb.position) <= maxDistSq

    if (isOnLeaderSight)
        steerForEvasion(out, boid, leader)

    // Creates a force to arrive at the behind point
    steerForArrival(_tmp1, boid, _behind)

    // Add separation force
    steerForFlock(_tmp2, boid, boids)

    vec2.add(out, out, _tmp1)
    return vec2.add(out, out, _tmp2)
}


export function steerForSeparation (out, boid, maxDistance, boidFieldOfView, flock) {
    // re-use the steerForSeparation function in vehicles. might work ok.
    VehicleSteering.steerForSeparation (out, boid, maxDistance, boidFieldOfView, flock)
}
