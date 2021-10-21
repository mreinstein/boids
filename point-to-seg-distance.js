import { vec2 } from './deps.js'


// temp static variables used to avoid mem allocations
const segmentNormal = [ 0, 0 ]
const local = [ 0, 0 ]


// this function was ported from OpenSteer's Pathway.cpp implementation

// computes distance from a point to a line segment
export default function pointToSegmentDistance (out, point, ep0, ep1) {

    vec2.subtract(segmentNormal, ep1, ep0)
    vec2.normalize(segmentNormal, segmentNormal)

    const segmentLength = vec2.distance(ep0, ep1)

    // convert the test point to be "local" to ep0
    vec2.subtract(local, point, ep0)

    // find the projection of "local" onto "segmentNormal"
    const segmentProjection = vec2.dot(segmentNormal, local)

    // handle boundary cases: when projection is not on segment, the
    // nearest point is one of the endpoints of the segment
    if (segmentProjection < 0) {
        vec2.copy(out.chosen, ep0)
        out.segmentProjection = 0
        out.distance = vec2.distance(point, ep0)
        return out
    }

    if (segmentProjection > segmentLength) {
        vec2.copy(out.chosen, ep1)
        out.segmentProjection = segmentLength
        out.distance = vec2.distance(point, ep1)
        return out
    }

    // otherwise nearest point is projection point on segment
    vec2.scale(out.chosen, segmentNormal, segmentProjection)
    vec2.add(out.chosen, out.chosen, ep0)

    out.segmentProjection = segmentProjection
    out.distance = vec2.distance(point, out.chosen)
    return out
}
