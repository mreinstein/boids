import { vec2 } from './deps.js'


// return component of vector parallel to a unit basis vector
// (IMPORTANT NOTE: assumes "basis" has unit magnitude (length==1))
function parallelComponent (out, unitBasis, force) {
    const projection = vec2.dot(force, unitBasis)
    return vec2.scale(out, unitBasis, projection)
}


// return component of vector perpendicular to a unit basis vector
// (IMPORTANT NOTE: assumes "basis" has unit magnitude (length==1))
function perpendicularComponent (out, unitBasis, force) {
    parallelComponent(out, unitBasis, force)
    return vec2.subtract(out, force, out)
}


// Does a "ceiling" or "floor" operation on the angle by which a given vector
// deviates from a given reference basis vector.  Consider a cone with "basis"
// as its axis and slope of "cosineOfConeAngle".  The first argument controls
// whether the "source" vector is forced to remain inside or outside of this
// cone.
//
// @param Object out vec2 that the limited source vector is copied into
// @param Bool insideOrOutside true means "source must be inside the cone", false means "source must be outside the cone" 
// @param Object source vec2 indicating vector to limit
// @param Number cosineOfConeAngle  one half of the full angle. e.g., if you want the cone to be 90 degrees,
//                                  cosineOfConeAngle = Math.cos(toRadians(90/2))
// @param Object basis normalized vec2 indicating the reference base vector
// @return Object vec2 resulting limited vector (the out parameter)
export default function limitDeviationAngle (out, insideOrOutside, source, cosineOfConeAngle, basis) {
    // immediately return zero length input vectors
    const sourceLength = vec2.length(source)
    if (sourceLength === 0)
        return vec2.copy(out, source)

    // measure the angular diviation of "source" from "basis"
    const direction = vec2.scale(vec2.create(), source, 1 / sourceLength)
    const cosineOfSourceAngle = vec2.dot(direction, basis)

    // Simply return "source" if it already meets the angle criteria.
    // (note: we hope this top "if" gets compiled out since the flag
    // is a constant when the function is inlined into its caller)
    if (insideOrOutside) {
        // source vector is already inside the cone, just return it
        if (cosineOfSourceAngle >= cosineOfConeAngle)
            return vec2.copy(out, source)

    } else {
        // source vector is already outside the cone, just return it
        if (cosineOfSourceAngle <= cosineOfConeAngle)
            return vec2.copy(out, source)
    }

    // find the portion of "source" that is perpendicular to "basis"
    const perp = perpendicularComponent(vec2.create(), basis, source)

    // normalize that perpendicular
    const unitPerp = vec2.normalize(vec2.create(), perp)

    // construct a new vector whose length equals the source vector,
    // and lies on the intersection of a plane (formed the source and
    // basis vectors) and a cone (whose axis is "basis" and whose
    // angle corresponds to cosineOfConeAngle)
    const perpDist = Math.sqrt(1 - (cosineOfConeAngle * cosineOfConeAngle))
    const c0 = vec2.scale(vec2.create(), basis, cosineOfConeAngle)
    const c1 = vec2.scale(vec2.create(), unitPerp, perpDist)

    vec2.add(out, c0, c1)
    return vec2.scale(out, out, sourceLength)
}
