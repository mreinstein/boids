import { vec2 }               from './deps.js'
import pointToSegmentDistance from './point-to-seg-distance.js'


// this module was ported from OpenSteer's Pathway.cpp implementation

// static temp variables used to avoid mem allocations
const tmpSegDistance = {
    segmentProjection: 0,
    distance: 0,
    chosen: [ 0, 0]
}


// a simple implementation of the Pathway protocol.  The path is a "polyline" a series of line
// segments between specified points.  A radius defines a volume for the path which is the union
// of a sphere at each point and a cylinder along each segment.
function create (options={}) {
    // construct a PolylinePathway given the number of points (vertices),
    // an array of points, and a path radius.
    let { points, pointCount, radius, cyclic } = options

    if (cyclic)
        pointCount++

    const normals = [ ]
    const lengths = [ ]
    let totalPathLength = 0

    // loop over all points
    for (let i = 0; i < pointCount; i++) {
        // copy in point locations, closing cycle when appropriate
        const closeCycle = cyclic && (i == pointCount-1)
        const j = closeCycle ? 0 : i
        points[i] = points[j]

        // for the end of each segment
        if (i > 0) {
            // compute the segment length
            normals[i] = vec2.subtract(vec2.create(), points[i], points[i-1])
            lengths[i] = vec2.length(normals[i])

            // find the normalized vector parallel to the segment
            vec2.normalize(normals[i], normals[i])

            // keep running total of segment lengths
            totalPathLength += lengths[i]
        }
    }

    return {
        pointCount,
        points,
        radius,
        cyclic,

        // lengths of each line segment between the points
        lengths,

        // unit vectors of each line segment between the points
        normals,

        totalPathLength
    }
}


// Given an arbitrary point ("A"), returns the nearest point ("P") on
// this path.  Also returns, via output arguments, the path tangent at
// P and a measure of how far A is outside the Pathway's "tube".  Note
// that a negative distance indicates A is inside the Pathway.
function mapPointToPath (out, path, point) {
    let d, minDistance

    // loop over all segments, find the one nearest to the given point
    for (let i = 1; i < path.pointCount; i++) {
        pointToSegmentDistance(tmpSegDistance, point, path.points[i-1], path.points[i])
        d = tmpSegDistance.distance
        if ((minDistance === undefined) || d < minDistance) {
            minDistance = d
            vec2.copy(out.onPath, tmpSegDistance.chosen) // set the point on the path

            vec2.subtract(out.tangent, path.points[i], path.points[i-1])
            vec2.normalize(out.tangent, out.tangent)
        }
    }

    // measure how far original point is outside the Pathway's "tube"
    out.outside = vec2.distance(out.onPath, point) - path.radius

    return out
}


// given an arbitrary point, convert it to a distance along the path
function mapPointToPathDistance (path, point) {
    let d, minDistance
    let segmentLengthTotal = 0
    let pathDistance = 0

    for (let i = 1; i < path.pointCount; i++) {
        pointToSegmentDistance(tmpSegDistance, point, path.points[i-1], path.points[i])
        d = tmpSegDistance.distance
        const segmentLength = path.lengths[i]

        if ((minDistance === undefined) || d < minDistance) {
            minDistance = d
            pathDistance = segmentLengthTotal + tmpSegDistance.segmentProjection
        }
        segmentLengthTotal += segmentLength
    }

    // return distance along path of onPath point
    return pathDistance
}


// given a distance along the path, convert it to a point on the path
function mapPathDistanceToPoint (out, path, pathDistance) {
    // clip or wrap given path distance according to cyclic flag
    let remaining = pathDistance

    if (path.cyclic) {
        remaining = pathDistance % path.totalPathLength //fmod(pathDistance, totalPathLength)
    } else {
        if (pathDistance < 0)
            return vec2.copy(out, path.points[0])
        if (pathDistance >= path.totalPathLength)
            return vec2.copy(out, path.points[path.pointCount-1])
    }

    // step through segments, subtracting off segment lengths until
    // locating the segment that contains the original pathDistance.
    // Interpolate along that segment to find 3d point value to return.
    for (let i = 1; i < path.pointCount; i++) {
        const segmentLength = path.lengths[i]
        if (segmentLength < remaining) {
            remaining -= segmentLength
        } else {
            const ratio = remaining / segmentLength
            //result = interpolate(ratio, points[i-1], points[i])
            vec2.lerp(out, path.points[i-1], path.points[i], ratio)
            break
        }
    }
    return out
}



export default { create, mapPointToPath, mapPointToPathDistance, mapPathDistanceToPoint }
