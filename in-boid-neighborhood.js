import { vec2 }         from './deps.js'
import conePointOverlap from 'https://cdn.jsdelivr.net/gh/mreinstein/collision-2d/src/cone-point-overlap.js'


// is a given vehicle within this boid's neighborhood?
export default function inBoidNeighborhood (boid, other, minDistance, maxDistance, boidFieldOfView) {
    if (other === boid)
        return false

    const rotation = Math.atan2(boid.rigidBody.velocity[1], boid.rigidBody.velocity[0])
    return conePointOverlap(boid.transform.position, rotation, boidFieldOfView, minDistance, maxDistance, other.transform.position)
}
