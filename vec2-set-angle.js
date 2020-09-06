import { vec2 } from './deps.js'


export default function setAngle (inp, angle) {
    const length = vec2.length(inp)
    inp[0] = Math.cos(angle) * length
    inp[1] = Math.sin(angle) * length
    return inp
}
