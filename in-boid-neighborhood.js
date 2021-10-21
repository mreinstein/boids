import { vec2 }         from './deps.js'
import conePointOverlap from 'https://cdn.jsdelivr.net/gh/mreinstein/collision-2d@a71f5e795c37d6ad501816c72e9e6edd0c5084c4/src/cone-point-overlap.js'


// is a given vehicle within this boid's neighborhood?
export default function inBoidNeighborhood (boid, other, minDistance, maxDistance, boidFieldOfView) {
    if (other === boid)
        return false

    const rotation = Math.atan2(boid.rigidBody.velocity[1], boid.rigidBody.velocity[0])
    return conePointOverlap(boid.transform.position, rotation, boidFieldOfView, minDistance, maxDistance, other.transform.position)
}
