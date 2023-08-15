import Alea                from 'https://cdn.skypack.dev/pin/alea@v1.0.0-P9lu4rchYeqab9T0CblM/mode=imports/optimized/alea.js'
import Path                from './polyline-path.js'
import Pool                from 'https://cdn.jsdelivr.net/gh/mreinstein/vec2-gap/pool.js'
import { clamp, vec2, segmentSphereOverlap, perpendicularComponent } from './deps.js'
import inBoidNeighborhood  from './in-boid-neighborhood.js'
import lerp                from 'https://cdn.skypack.dev/pin/lerp@v1.0.3-fjXpN7X7nWMDOQ8ERPDB/mode=imports,min/optimized/lerp.js'
import limitDeviationAngle from './limit-deviation-angle.js'
import vec2SetLength       from 'https://cdn.jsdelivr.net/gh/mreinstein/vec2-gap/set-length.js'
import vec2Truncate        from 'https://cdn.jsdelivr.net/gh/mreinstein/vec2-gap/truncate.js'


const defaultSeed = Math.random()
const defaultRng = new Alea(defaultSeed)

const ORIGIN = vec2.fromValues(0, 0)

const _desiredVelocity = vec2.create()
const _offset = vec2.create()

const contact = { }


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


// apply a given steering force to our momentum,
// adjusting our orientation to maintain velocity-alignment.
//
// @param Number elapsedTime  seconds elapsed (dt)
export function applySteeringForce (boid, force, elapsedTime) {
    
    const adjustedForce = adjustRawSteeringForce(vec2.create(), boid, force)

    // enforce limit on magnitude of steering force
    const clippedForce = vec2Truncate(vec2.create(), adjustedForce, boid.steering.maxForce)

    const newAcceleration = vec2.scale(vec2.create(), clippedForce, 1/boid.rigidBody.mass)

    // damp out abrupt changes and oscillations in steering acceleration
    // (rate is proportional to time step, then clipped into useful range)
    if (elapsedTime > 0) {
        const smoothRate = clamp(9 * elapsedTime, 0.15, 0.4)
        vec2.lerp(boid.rigidBody.smoothedAcceleration, boid.rigidBody.smoothedAcceleration, newAcceleration, smoothRate)
    }
    
    // Euler integrate (per frame) acceleration into velocity
    const newVelocity = vec2.create()

    newVelocity[0] = boid.rigidBody.velocity[0] + boid.rigidBody.smoothedAcceleration[0] * elapsedTime
    newVelocity[1] = boid.rigidBody.velocity[1] + boid.rigidBody.smoothedAcceleration[1] * elapsedTime

    // enforce speed limit
    vec2Truncate(boid.rigidBody.velocity, newVelocity, boid.rigidBody.maxSpeed)

    // integrating position is disabled intentionally. In cases where a rigidBody physics system performs collision response,
    // updating position here directly interferes with that.

    // Euler integrate (per frame) velocity into position
    //vec2.scaleAndAdd(boid.transform.position, boid.transform.position, boid.rigidBody.velocity, elapsedTime)
}


// from opensteer. just tries to keep the vehicle on the path
// The path defines a tube in terms of a spine and a radius, the goal is to keep a
// vehicle headed toward a point inside that tube.
export function steerToStayOnPath (out, boid, path, predictionTime) {
    // predict our future position
    const futurePosition = predictFuturePosition(vec2.create(), boid, predictionTime)

    // find the point on the path nearest the predicted future position
    const result = {
        onPath: vec2.create(),
        tangent: vec2.create(),
        outside: 0.0
    }

    Path.mapPointToPath(result, path, futurePosition)

    if (result.outside < 0)
    {
        // our predicted future position was in the path, return zero steering.
        return vec2.set(out, 0, 0)
    }
    else
    {
        // our predicted future position was outside the path, need to
        // steer towards it.  Use onPath projection of futurePosition
        // as seek target
        if (window.annotatePathFollowing)
            annotatePathFollowing(boid.transform.position, futurePosition, onPath, onPath, outside)

        return steerForSeek(out, boid, result.onPath)
    }
}


function predictFuturePosition (out, boid, predictionTime) {
    vec2.scale(out, boid.rigidBody.velocity, predictionTime)
    return vec2.add(out, out, boid.transform.position)
}


// steerToFollowPath provides directed path following where the vehicle both stays on the path and
// heads in a given direction along the path, as indicated by the direction argument which should be
// either +1 or -1. The path defines a tube in terms of a spine and a radius, the goal is to keep a
// vehicle headed toward a point inside that tube.
export function steerForFollowPath (out, boid, path, direction, predictionTime, loop=false) {

    // our goal will be offset from our path distance by this amount
    const pathDistanceOffset = direction * predictionTime * vec2.length(boid.rigidBody.velocity)

    // predict our future position
    const futurePosition = predictFuturePosition(vec2.create(), boid, predictionTime)
    
    // measure distance along path of our current and predicted positions
    const nowPathDistance = Path.mapPointToPathDistance(path, boid.transform.position)

    const futurePathDistance = Path.mapPointToPathDistance(path, futurePosition)

    // are we facing in the correction direction?
    const rightway = (pathDistanceOffset > 0) ?
                           (nowPathDistance < futurePathDistance) :
                           (nowPathDistance > futurePathDistance)

    // find the point on the path nearest the predicted future position
    const result = {
        onPath: vec2.create(),
        tangent: vec2.create(),
        outside: 0.0
    }

    Path.mapPointToPath(result, path, futurePosition)

    // no steering is required if (a) our future position is inside
    // the path tube and (b) we are facing in the correct direction
    if ((result.outside < 0) && rightway)
    {
        return vec2.set(out, 0, 0) // all is well, return zero steering
    }
    else
    {
        // otherwise we need to steer towards a target point obtained
        // by adding pathDistanceOffset to our current path position

        const targetPathDistance = nowPathDistance + pathDistanceOffset
        const target = Path.mapPathDistanceToPoint(vec2.create(), path, targetPathDistance)

        if (window.annotatePathFollowing)
            annotatePathFollowing(boid.transform.position, futurePosition, result.onPath, target, result.outside)

        //console.log('tgt:', target, 'tpd:', targetPathDistance)

        // return steering to seek target on path
        return steerForSeek(out, boid, target)
    }
}


// steer away from targetVec
export function steerForFlee (out, fleeingBoid, targetVec) {
    vec2.subtract(_desiredVelocity, fleeingBoid.aabb.position, targetVec)
    return vec2.subtract(out, _desiredVelocity, fleeingBoid.rigidBody.velocity)
}


export function xxxSteerForFlee (out, fleeingBoid, targetVec) {
    vec2.subtract(_offset, fleeingBoid.aabb.position, targetVec)
    vec2Truncate(_desiredVelocity, _offset, boid.rigidBody.maxSpeed)
    return vec2.subtract(out, _desiredVelocity, fleeingBoid.rigidBody.velocity)
}


// steer away from neighbors
export function steerForSeparation (out, boid, maxDistance, boidFieldOfView, flock) {
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
// from plugins/Boids.cpp
export function steerForFlock (out, boid, flock) {
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




// TODO: evaluate all the functions below this line and figure out if they belong in arcade or vehicle


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


// @param Object out vec2 of resulting steering force
// @param Obecjt boid
// @param Array boids obstacles to avoid (other boids)
function steerForCollisionAvoidance (out, boid, boids) {
    const ahead = vec2.normalize(Pool.malloc(), boid.rigidBody.velocity)

    const t = vec2.length(boid.rigidBody.velocity) / boid.rigidBody.maxSpeed

    vec2.scaleAndAdd(ahead, boid.aabb.position, ahead, boid.steering.MAX_SEE_AHEAD * t)

    const ahead2 = vec2.normalize(Pool.malloc(), boid.rigidBody.velocity)
    vec2.scaleAndAdd(ahead2, boid.aabb.position, ahead2, boid.steering.MAX_SEE_AHEAD * 0.5 * t)

    const mostThreatening = _findMostThreateningObstacle(boid, boids, ahead, ahead2)

    if (mostThreatening) {
        // compute avoidance steering force: take offset from obstacle to me,
        const offset = vec2.subtract(vec2.create(), boid.aabb.position,  mostThreatening.aabb.position)

        // take the component of that which is lateral (perpendicular to my forward direction),
        const forward = vec2.normalize(vec2.create(), boid.rigidBody.velocity)
        const avoidance = perpendicularComponent(vec2.create(), forward, offset)

        //  set length to maxForce, add a bit of forward component
        vec2.normalize(avoidance, avoidance)
        vec2.scale(avoidance, avoidance, boid.steering.maxForce)

        //vec2.copy(out, avoidance)
        vec2.scaleAndAdd(out, avoidance, forward, boid.steering.maxForce * 0.75)
        
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

    const tmp = [ 0, 0 ]
    const forward = vec2.normalize([ 0, 0 ], boid.rigidBody.velocity)

    for (const next of boids) {
        if (next === boid)
            continue

        const radius = boid.steering.radius || Math.max(next.aabb.width, next.aabb.height)

        const p2 = vec2.scaleAndAdd(vec2.create(), boid.aabb.position, forward, boid.steering.MAX_SEE_AHEAD)
        const intersects = segmentSphereOverlap(boid.aabb.position, p2, next.aabb.position, next.steering.radius, contact)

        if (intersects && (!mostThreatening || vec2.distance(boid.aabb.position, next.aabb.position) < vec2.distance(boid.aabb.position, mostThreatening.aabb.position)))
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
