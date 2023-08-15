import { createSteeringComponent } from '../src/steering-arcade.js'
import segmentNormal from 'https://cdn.jsdelivr.net/gh/mreinstein/collision-2d/src/segment-normal.js'
import { vec2 }      from 'https://cdn.skypack.dev/pin/gl-matrix@v3.4.3-OSmwlRYK5GW1unkuAQkN/mode=imports,min/optimized/gl-matrix.js'


export function createBoid ({ x, y, maxSpeed, maxForce, radius, arrivalThreshold, maxSeeAhead }) {
    const transform = {
        position: [ x, y ],
        rotation: 0
    }

    return {
        aabb: {
            width: 10,
            height: 15,
            position: transform.position
        },

        rigidBody: {
            maxSpeed,             // maximum speed vehicle is allowed to move in forward direction
            velocity: [ 0, 0 ],
            mass: 1,                   // mass (defaults to 1 so acceleration=force)

            smoothedAcceleration: [ 0, 0 ]
        },

        steering: createSteeringComponent({ maxForce, radius, arrivalThreshold, maxSeeAhead }),

        transform
    }
}


export function drawBoid (boid, context, boidColor='#333') {
    
    context.save()

    const halfWidth = boid.aabb.width / 2
    const halfHeight = boid.aabb.height / 2
    const translateX = boid.aabb.position[0]
    const translateY = boid.aabb.position[1]

    // handle rotation
    context.translate(translateX, translateY)

    context.rotate(boid.transform.rotation)

    //context.translate(-halfWidth, -halfHeight)

    context.fillStyle = boidColor

    context.beginPath()

    context.moveTo(halfWidth, 0)
    context.lineTo(-halfWidth, -halfHeight*0.6)
    context.lineTo(-halfWidth, halfHeight*0.6)
    context.lineTo(halfWidth, 0)

    context.closePath()
    context.fill()

    context.restore()
}


export function drawTarget (target, context) {
    if (!target)
        return

    context.strokeStyle = 'deeppink'
    context.lineWidth = 1
    const radius = 6

    context.beginPath()
    context.arc(target[0], target[1], radius, 0, 2 * Math.PI, true)
    context.closePath()
    context.stroke()

    context.strokeStyle = 'dodgerblue'
    context.beginPath()
    context.moveTo(target[0] + 0.5, target[1]-12)
    context.lineTo(target[0] + 0.5, target[1]+12)

    context.moveTo(target[0] - 12.5, target[1] + 0.5)
    context.lineTo(target[0] + 12.5, target[1] + 0.5)
   
    context.closePath()
    context.stroke()    
}


export function drawPathTube (path, canvas, context) {
    context.fillStyle = '#ccc'
    for (const pt of path.points) {
        context.beginPath()
        context.arc(pt[0], pt[1], path.radius, 0, 2 * Math.PI, true)
        context.closePath()
        context.fill()
    }

    // draw oriented boxes to connect the end caps
    for (let i=1; i < path.pointCount; i++) {
        const p0 = path.points[i-1]
        const p1 = path.points[i]

        const normal = [ 0, 0 ]

        context.beginPath()

        segmentNormal(normal, p0, p1)
        vec2.scale(normal, normal, path.radius)

        context.moveTo(p0[0] + normal[0], p0[1] + normal[1])
        context.lineTo(p1[0] + normal[0], p1[1] + normal[1])
        vec2.scale(normal, normal, -1)
        context.lineTo(p1[0] + normal[0], p1[1] + normal[1])
        context.lineTo(p0[0] + normal[0], p0[1] + normal[1])

        vec2.scale(normal, normal, -1)
        context.lineTo(p0[0] + normal[0], p0[1] + normal[1])

        context.closePath()
        context.fill()
    }
}


export function drawPath (path, canvas, context) {
    context.beginPath()
    context.strokeStyle = '#333'
    for (let i=1; i < path.pointCount; i++) {
        const p0 = path.points[i-1]
        const p1 = path.points[i]
        context.moveTo(p0[0]+ 0.5, p0[1] + 0.5)
        context.lineTo(p1[0] + 0.5, p1[1] + 0.5)
    }

    context.closePath()
    context.stroke()

    context.fillStyle = 'dodgerblue'
    for (const pt of path.points) {
        context.fillRect(pt[0]-2, pt[1]-2, 4, 4)
    }
}
