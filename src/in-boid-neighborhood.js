import { conePointOverlap, vec2 } from './deps.js'


// is a given vehicle within this boid's neighborhood?
// @param number boidFieldOfView angle of cone in degrees
export default function inBoidNeighborhood (boid, other, minDistance, maxDistance, boidFieldOfView) {
    if (other === boid)
        return false

    const rotation = Math.atan2(boid.rigidBody.velocity[1], boid.rigidBody.velocity[0])
    return conePointOverlap(boid.transform.position, rotation, boidFieldOfView, minDistance, maxDistance, other.transform.position)
}
