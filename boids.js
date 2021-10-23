import Alea                from 'https://cdn.skypack.dev/alea'
import Pool                from 'https://cdn.jsdelivr.net/gh/mreinstein/vec2-gap/pool.js'
import { vec2 }            from './deps.js'
import inBoidNeighborhood  from './in-boid-neighborhood.js'
import lerp                from 'https://cdn.skypack.dev/lerp'
import limitDeviationAngle from './limit-deviation-angle.js'
import vec2SetLength       from 'https://cdn.jsdelivr.net/gh/mreinstein/vec2-gap/set-length.js'
import vec2Truncate        from 'https://cdn.jsdelivr.net/gh/mreinstein/vec2-gap/truncate.js'


const defaultSeed = Math.random()
const defaultRng = new Alea(defaultSeed)

const ORIGIN = vec2.fromValues(0, 0)

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


function createSteeringComponent (options={}) {
    return {
        // common vehicle steering parameters
        maxForce: options.maxForce || 60,

        // collision avoidance
        MAX_SEE_AHEAD: options.maxSeeAhead || 100,

        // flock
        radius: options.radius || 10, // radius of sphere encapsulating collision body
        maxDistance: 300,
        minDistance: 60,

        // wander
        wanderDistance: 10,
        wanderRadius: 5,
        wanderAngle: 0,      // MUTABLE
        wanderRange: 1,

        // arrive
        arriveThreshold: 50,

        // leader follow
        leaderBehindDistance: 32,
        leaderSightRadius: 200,

        // path follow
        pathIndex: 0,
        pathThreshold: (options.pathThreshold === undefined) ? 20 : options.pathThreshold,

        // queueing
        maxQueueAhead: 40,
        maxQueueRadius: 20,
    }
}


// ----------------------------------------------------------------------------
// adjust the steering force passed to applySteeringForce.
//
// allows a specific vehicle class to redefine this adjustment.
// default is to disallow backward-facing steering at low speed.
function adjustRawSteeringForce (out, boid, force) {
    const maxAdjustedSpeed = 0.2 * boid.rigidBody.maxSpeed

    const speed = vec2.length(boid.rigidBody.velocity)

    if ((speed > maxAdjustedSpeed) || (vec2.length(force) == 0)) {
        return vec2.copy(out, force)

    } else {
        const range = speed / maxAdjustedSpeed
        // const float cosine = interpolate (pow (range, 6), 1.0f, -1.0f);
        // const float cosine = interpolate (pow (range, 10), 1.0f, -1.0f);
        // const float cosine = interpolate (pow (range, 20), 1.0f, -1.0f);
        // const float cosine = interpolate (pow (range, 100), 1.0f, -1.0f);
        // const float cosine = interpolate (pow (range, 50), 1.0f, -1.0f);
        const t = Math.pow(range, 20)

        const cosine = lerp(1.0, -1.0, t)
        const forward = (vec2.length(boid.rigidBody.velocity) === 0) ? [ 1, 0 ] : vec2.normalize([ 0, 0 ], boid.rigidBody.velocity)
        const insideCone = true

        return limitDeviationAngle(out, insideCone, force, cosine, forward)
    }
}


// NOTE: the per frame accelation and velocity is intentionally disabled.
//       It snaps more, but I think this looks better for enemies like bats, etc.
//       for things like vehicles that have more gradual acceleration the per-frame stuff might be better?
function applySteeringForce (boid, force/*, elapsedTime*/) {

    const adjustedForce = adjustRawSteeringForce(vec2.create(), boid, force)
    //const adjustedForce = vec2.copy(vec2.create(), force)

    // enforce limit on magnitude of steering force
    const clippedForce = vec2Truncate(vec2.create(), adjustedForce, boid.steering.maxForce)

    const newAcceleration = vec2.scale(vec2.create(), clippedForce, 1/boid.rigidBody.mass)

    // damp out abrupt changes and oscillations in steering acceleration
    const smoothRate = 0.4

    vec2.lerp(boid.rigidBody.smoothedAcceleration, boid.rigidBody.smoothedAcceleration, newAcceleration, smoothRate)


    // Euler integrate (per frame) acceleration into velocity
    //const newVelocity = vec2.scaleAndAdd(vec2.create(), boid.rigidBody.velocity, boid.rigidBody.smoothedAcceleration, elapsedTime)
    const newVelocity = vec2.add(vec2.create(), boid.rigidBody.velocity, boid.rigidBody.smoothedAcceleration)


    // enforce speed limit
    vec2Truncate(boid.rigidBody.velocity, newVelocity, boid.rigidBody.maxSpeed)


    // integrating position is disabled intentionally. In cases where a rigidBody physics system performs collision response, updating
    // position here directly interferes with that.

    // Euler integrate (per frame) velocity into position
    //vec2.scaleAndAdd(boid.transform.position, boid.transform.position, boid.rigidBody.velocity, elapsedTime)
    //vec2.add(boid.transform.position, boid.transform.position, boid.rigidBody.velocity)
}


// steer torwards target
function steerForSeek (out, boid, target) {
    const desiredVelocity = vec2.subtract(Pool.malloc(), target, boid.aabb.position)
    vec2SetLength(desiredVelocity, desiredVelocity, boid.rigidBody.maxSpeed)

    vec2.subtract(out, desiredVelocity, boid.rigidBody.velocity)

    Pool.free(desiredVelocity)
    return out
}


// seek until within arriveThreshold
function steerForArrival (out, boid, target) {
    const desiredVelocity = vec2.subtract(Pool.malloc(), target, boid.aabb.position)
    vec2.normalize(desiredVelocity, desiredVelocity)

    const distance = vec2.distance(boid.aabb.position, target)
    //const arriveThresholdSq = boid.steering.arriveThreshold * boid.steering.arriveThreshold
    //const distanceSq = vec2.squaredDistance(boid.aabb.position, target)
    //if (distanceSq > arriveThresholdSq) {
    if (distance > boid.steering.arriveThreshold) {
        vec2.scale(desiredVelocity, desiredVelocity, boid.rigidBody.maxSpeed)
    } else {
        //const scalar = boid.rigidBody.maxSpeed * distance / arriveThresholdSq
        const scalar = boid.rigidBody.maxSpeed * distance / boid.steering.arriveThreshold
        vec2.scale(desiredVelocity, desiredVelocity, scalar)
    }

    vec2.subtract(out, desiredVelocity, boid.rigidBody.velocity)

    Pool.free(desiredVelocity)
    return out
}


// follow a path made up of an array or vectors
function steerForFollowPath (out, boid, path, loop=false) {
    const wayPoint = path.points[boid.steering.pathIndex]
    if (!wayPoint) {
        boid.steering.pathIndex = 0
        return
    }

    // do we really want sqaured?
    //const pathThresholdSq = boid.steering.pathThreshold * boid.steering.pathThreshold
    //if (vec2.squaredDistance(boid.transform.position, wayPoint) < pathThresholdSq) {
    if (vec2.distance(boid.transform.position, wayPoint) < boid.steering.pathThreshold) {
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


// steer away from neighbors
function steerForSeparation (out, boid, maxDistance, boidFieldOfView, flock) {
    // steering accumulator and count of neighbors, both initially zero
    vec2.set(out, 0, 0)
    let neighbors = 0

    // for each of the other vehicles...
    const minDistance = boid.steering.radius // * 3
    for (const other of flock) {
        if (inBoidNeighborhood(boid, other, minDistance, maxDistance, boidFieldOfView)) {
            // add in steering contribution
            // (opposite of the offset direction, divided once by distance
            // to normalize, divided another time to get 1/d falloff)
            const offset = vec2.subtract([0,0], other.transform.position, boid.transform.position)
            const distanceSquared = vec2.dot(offset, offset)
            
            vec2.scaleAndAdd(out, out, offset, 1 / -distanceSquared)

            // count neighbors
            neighbors++
        }
    }

    // divide by neighbors, then normalize to pure direction
    if (neighbors > 0) {
        vec2.scale(out, out, 1 / neighbors)
        vec2.normalize(out, out)
    }

    return out
}


// Alignment behavior: steer to head in same direction as neighbors
function steerForAlignment (out, boid, maxDistance, boidFieldOfView, flock) {
    // steering accumulator and count of neighbors, both initially zero
    vec2.set(out, 0, 0)
    let neighbors = 0

    // for each of the other vehicles...
    const minDistance = boid.steering.radius * 3
    for (const other of flock) {
        if (inBoidNeighborhood(boid, other, minDistance, maxDistance, boidFieldOfView)) {
            // accumulate sum of neighbor's heading
            const forward = vec2.normalize([0,0], other.rigidBody.velocity)
            vec2.add(out, out, forward)

            // count neighbors
            neighbors++
        }
    }

    // divide by neighbors, subtract off current heading to get error-
    // correcting direction, then normalize to pure direction
    if (neighbors > 0) {
        const forward = vec2.normalize([0,0], boid.rigidBody.velocity)
        vec2.scale(out, out, 1 / neighbors)
        vec2.subtract(out, out, forward)
        vec2.normalize(out, out)
    }

    return out
}


// Cohesion behavior: to to move toward center of neighbors
function steerForCohesion (out, boid, maxDistance, boidFieldOfView, flock) {
    // steering accumulator and count of neighbors, both initially zero
    vec2.set(out, 0, 0)
    let neighbors = 0

    // for each of the other vehicles...
    const minDistance = boid.steering.radius * 3
    for (const other of flock) {
        if (inBoidNeighborhood(boid, other, minDistance, maxDistance, boidFieldOfView)) {
            // accumulate sum of neighbor's positions
            vec2.add(out, out, other.transform.position)

            // count neighbors
            neighbors++
        }
    }

    // divide by neighbors, subtract off current position to get error-
    // correcting direction, then normalize to pure direction
    if (neighbors > 0) {
        vec2.scale(out, out, 1 / neighbors)
        vec2.subtract(out, out, boid.transform.position)
        vec2.normalize(out, out)
    }

    return out
}


// group of boids loosely move together
function steerForFlock (out, boid, flock) {
    const separationRadius =  50.0
    //const separationAngle  = -0.707
    const separationFieldOfView = 135
    const separationWeight =  12.0

    const alignmentRadius = 60
    //const alignmentAngle  = 0.7
    const alignmentFieldOfView = 46
    const alignmentWeight = 8.0

    const cohesionRadius = 20
    //const cohesionAngle  = -0.15
    const cohesionFieldOfView = 99
    const cohesionWeight = 8.0

    const maxRadius = Math.max(separationRadius, alignmentRadius, cohesionRadius)

    // determine each of the three component behaviors of flocking
    const separation = steerForSeparation([ 0, 0 ], boid, separationRadius, separationFieldOfView, flock)

    const alignment  = steerForAlignment([ 0, 0 ], boid, alignmentRadius, alignmentFieldOfView, flock)
    
    const cohesion   = steerForCohesion([ 0, 0 ], boid, cohesionRadius, cohesionFieldOfView, flock)

    // apply weights to components (save in variables for annotation)
    const separationW = vec2.scale([ 0, 0 ], separation, separationWeight)
    const alignmentW = vec2.scale([ 0, 0 ], alignment, alignmentWeight)
    const cohesionW = vec2.scale([ 0, 0 ], cohesion, cohesionWeight)

    vec2.add(out, separationW, alignmentW)
    return vec2.add(out, out, cohesionW) 
}


// wander around, changing angle by a limited amount each tick
function steerForWander (out, boid, random=defaultRng) {
    const center = Pool.malloc()
    vec2.normalize(center, boid.rigidBody.velocity)
    vec2.scale(center, center, boid.steering.wanderDistance)

    const offset = Pool.malloc(boid.steering.wanderRadius, 0)
    vec2.rotate(offset, offset, ORIGIN, boid.steering.wanderAngle)

    boid.steering.wanderAngle += random() * boid.steering.wanderRange - boid.steering.wanderRange * 0.5

    vec2.add(out, center, offset)
    
    Pool.free(offset)
    Pool.free(center)

    return out
}


// look at velocity of target boid and try to predict where it's going with the aim of catching it
function steerForPursuit (out, pursuerBoid, targetPosition, targetVelocity) {

    const maxSpeedSq = pursuerBoid.rigidBody.maxSpeed * pursuerBoid.rigidBody.maxSpeed
    const lookAheadTime = vec2.squaredDistance(pursuerBoid.aabb.position, targetPosition) / maxSpeedSq

    const scaledVelocity = Pool.malloc()
    vec2.scale(scaledVelocity, targetVelocity, lookAheadTime)

    const predictedTarget = Pool.malloc()

    vec2.add(predictedTarget, targetPosition, scaledVelocity)

    steerForSeek(out, pursuerBoid, predictedTarget)

    Pool.free(scaledVelocity)
    Pool.free(predictedTarget)

    return out
}


// steer away from targetVec
function steerForFlee (out, fleeingBoid, targetVec) {
    const desiredVelocity = Pool.malloc()
    vec2.subtract(desiredVelocity, targetVec, fleeingBoid.aabb.position)
    vec2.normalize(desiredVelocity, desiredVelocity)
    vec2.scale(desiredVelocity, desiredVelocity, fleeingBoid.rigidBody.maxSpeed)

    vec2.subtract(out, desiredVelocity, fleeingBoid.rigidBody.velocity)
    vec2.negate(out, out)
    
    Pool.free(desiredVelocity)

    return out
}


// look at velocity of boid and try to predict where it's going
// @param Object menaceBoid the boid to evade
function steerForEvasion (out, evadingBoid, menaceBoid) {
    const maxSpeedSq = evadingBoid.rigidBody.maxSpeed * evadingBoid.rigidBody.maxSpeed
    const lookAheadTime = vec2.squaredDistance(evadingBoid.aabb.position, menaceBoid.aabb.position) / maxSpeedSq

    const scaledVelocity = Pool.malloc()
    vec2.scale(scaledVelocity, menaceBoid.rigidBody.velocity, lookAheadTime)

    const predictedTarget = Pool.malloc()
    vec2.add(predictedTarget, menaceBoid.aabb.position, scaledVelocity)

    steerForFlee(out, evadingBoid, predictedTarget)

    Pool.free(scaledVelocity)
    Pool.free(predictedTarget)

    return out
}


// http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-leader-following--gamedev-10810
function steerForFollowLeader (out, boid, leader, boids) {
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
        steerForEvasion(out, boid, leader)

    // Creates a force to arrive at the behind point
    const tmp1 = steerForArrival(Pool.malloc(), boid, behind)

    // Add separation force
    const tmp2 = steerForFlock(Pool.malloc(), boid, boids)

    vec2.add(out, out, tmp1)
    vec2.add(out, out, tmp2)

    Pool.free(ahead)
    Pool.free(tv)
    Pool.free(behind)
    Pool.free(tmp1)
    Pool.free(tmp2)

    return out
}


// http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-queue--gamedev-14365
function steerForQueueing (out, boid, boids) {
    const neighbor = _getNeighborAhead(boid, boids)
    if (!neighbor)
        return

    const brake = Pool.malloc()
    vec2.negate(brake, boid.steering.steeringForce)
    vec2.scale(brake, brake, 0.8)

    const v = Pool.malloc()
    vec2.negate(v, boid.rigidBody.velocity)

    vec2.add(brake, brake, v)
    vec2.add(brake, brake, steerForFlock(out, boid, boids))

    // TODO: use steering force here rather than adjust velocity directly
    if (vec2.distance(object.aabb.position, neighbor.aabb.position) <= boid.steering.maxQueueRadius)
      vec2.scale(boid.rigidBody.velocity, boid.rigidBody.velocity, 0.3)

    vec2.add(out, out, brake)

    Pool.free(v)
    Pool.free(brake)

    return out
}


// http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
// @param Map boids
function steerForCollisionAvoidance (out, boid, boids) {
    const ahead = vec2.normalize(Pool.malloc(), boid.rigidBody.velocity)
    vec2.scaleAndAdd(ahead, boid.aabb.position, ahead, boid.steering.MAX_SEE_AHEAD)

    const ahead2 = vec2.normalize(Pool.malloc(), boid.rigidBody.velocity)
    vec2.scaleAndAdd(ahead2, boid.aabb.position, ahead2, boid.steering.MAX_SEE_AHEAD * 0.5)

    const mostThreatening = _findMostThreateningObstacle(boid, boids, ahead, ahead2)

    if (mostThreatening) {
        // obstacle, avoidance force needed
        const avoidance = vec2.subtract(Pool.malloc(), ahead, mostThreatening.aabb.position)
        vec2.normalize(avoidance, avoidance)
        vec2.scaleAndAdd(out, out, avoidance, boid.steering.maxForce)
        Pool.free(avoidance)
    } else {
        vec2.set(out, 0, 0)
    }

    Pool.free(ahead)
    Pool.free(ahead2)

    return out
}


// @param Map objects
// @param vec2 ahead
// @param vec2 ahead2
function _findMostThreateningObstacle (boid, boids, ahead, ahead2) {
    let mostThreatening

    for (const next of boids) {
        if (next === boid)
            continue
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


// TODO: replace this check, it sucks. It will only detect intersections when the endpoint of the line are within the circle
// the property "center" of the obstacle is a vec2
function _lineIntersectsCircle (ahead, ahead2, center, radius) {
    return vec2.distance(center, ahead) <= radius || vec2.distance(center, ahead2) <= radius
}


export default {
    createSteeringComponent,

    applySteeringForce,
    steerForSeparation,
    steerForAlignment,
    steerForCohesion,
    steerForFlock,

    steerForArrival, steerForEvasion, steerForFlee, steerForFollowLeader, steerForFollowPath,
    steerForPursuit, steerForQueueing, steerForSeek, steerForWander, steerForCollisionAvoidance
}
