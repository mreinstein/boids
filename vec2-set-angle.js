import * as vec2 from 'https://cdn.jsdelivr.net/npm/gl-matrix@3.3.0/esm/vec2.js'


export default function setAngle (inp, angle) {
    const length = vec2.length(inp)
    inp[0] = Math.cos(angle) * length
    inp[1] = Math.sin(angle) * length
    return inp
}
