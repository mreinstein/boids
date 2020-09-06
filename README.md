# boids

functional, data-oriented steering behaviors


## why

I had trouble finding steering behaviors with the following properties:

* functional and data oriented
* doesn't create memory garbage

So here we are.


## usage

```javascript
import steering from 'https://cdn.jsdelivr.net/gh/mreinstein/boids/boids.js'


const b = {
    steering: steering.createSteeringComponent()
    aabb: {
        width: 10,
        height: 10,
        position: [ 50, 50 ]
    },
    rigidBody: {
        // TODO: fill in details here
    }
}

// apply steering behaviors here...
// steerForSeek(b, [ 300, 300 ])
```

TODO: expand usage examples here


## inspiration
* http://www.red3d.com/cwr/steer/gdc99/  (original concept author)
* https://github.com/ianmcgregor/boid    (high quality but object oriented javascript implementation)
* https://gamedevelopment.tutsplus.com/series/understanding-steering-behaviors--gamedev-12732
