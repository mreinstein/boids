# boids

functional, data-oriented steering behaviors


## why

I had trouble finding steering behaviors with the following properties:

* functional and data oriented
* doesn't create memory garbage
* re-uses a robust vector module rather than re-inventing the wheel


So here we are.


## usage

There are 2 separate steering implementations:

* **Arcade Steering** is velocity only. no acceleration. This tends to work well for units that are moved using arcade physics, with snappy velocity changes
* **Vehicle Steering** uses velocity and acceleration. It is smoothed motion, and works well for vehicles which tend to accelerate/deccelerate more gradually than arcade units.

The vehicle steering is adapted from OpenSteer, and the arcade steering is adapted from Ian McGregor's open source module.
Hence the differences in available steering behaviors


TODO: put a table here showing which implementations support which steering behaviors

| function  |  vehicle | arcade |
| --------------------------------- | ---------------------------------------- | ---------------------------------------- |
| blah                              |   yes                                    |    no                                    |



```javascript
import * as Boids from 'https://cdn.jsdelivr.net/gh/mreinstein/boids/src/steering-arcade.js'
import { vec2 }   from 'https://cdn.skypack.dev/pin/gl-matrix@v3.4.3-OSmwlRYK5GW1unkuAQkN/mode=imports,min/optimized/gl-matrix.js'


const position = [ 50, 50 ]

const b = {
    steering: Boids.createSteeringComponent()
    aabb: {
        width: 10,
        height: 10,
        position
    },
    transform: {
        position
    },
    rigidBody: {
        mass: 1,
        maxSpeed: 5, // the max length of the velocity vector
        smoothedAcceleration: [ 0, 0, ], // internal data structure used to smooth accel over time
        velocity: [ 0, 0 ]
    }
}

const steeringForce = vec2.create()


function drawBoid (b) {
    // omitted for brevity, draw the boid using whatever graphics API you're using here
}


function stepBoid (b) {
    // reset the steering force vector used every frame.
    vec2.set(steeringForce, 0, 0)

    // determine  steering forces
    Boids.steerForSeek(steeringForce, b, [ 300, 300 ])

    // determine the new velocity for the boid based on a steering force
    Boids.applySteeringForce(b, steeringForce)


    // Euler integrate (per frame) velocity into position
    vec2.add(boid.transform.position, boid.transform.position, boid.rigidBody.velocity)
}


function tick () {
    stepBoid(b)
    drawBoid(b)
    requestAnimationFrame(tick) 
}


requestAnimationFrame(tick)

```

The `examples/` directory has runnable demos of all steering behaviors.


## available steering behaviors

* `steerForSeparation`
* `steerForAlignment`
* `steerForCohesion`
* `steerForSeparation`
* `steerForFlock`
* `steerForArrival`
* `steerForEvasion`
* `steerForFlee`
* `steerForFollowLeader`
* `steerForFollowPath`
* `steerForPursuit`
* `steerForQueueing`
* `steerForSeek`
* `steerForWander`
* `steerForCollisionAvoidance`


## inspiration
* http://www.red3d.com/cwr/steer/gdc99/  Craig Reynold's seminal work (original concept author)
* https://github.com/ianmcgregor/boid    (high quality but object oriented javascript implementation)
* https://gamedevelopment.tutsplus.com/series/understanding-steering-behaviors--gamedev-12732
