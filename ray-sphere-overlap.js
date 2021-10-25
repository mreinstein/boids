// adapted from https://github.com/mattdesl/ray-sphere-intersection/blob/master/index.js
import { vec2 }               from './deps.js'


const _tmp = [ 0, 0 ]


// @param Object out vec2 containing intersection point
// @param Object origin vec2 of ray starting point
// @param Object direction vec2 normalized ray direction
// @param Object center vec2 center point of sphere
// @param Number radius sphere radius
// @returns Boolean true if there's an intersection, false otherwise
export default function intersectRaySphere (out, origin, direction, center, radius) {
    vec2.subtract(_tmp, center, origin)
    const len = vec2.dot(direction, _tmp)
    if (len < 0) // sphere is behind ray
        return false

    vec2.scaleAndAdd(_tmp, origin, direction, len)
    const dSq = vec2.squaredDistance(center, _tmp)
    const rSq = radius * radius
    if (dSq > rSq)
        return false

    vec2.scale(out, direction, len - Math.sqrt(rSq - dSq))
    vec2.add(out, out, origin)
    return true
}
