import { vec2, setLength } from './deps.js'


export default function truncate (inp, maxLength) {
    if (vec2.length(inp) > maxLength)
        setLength(inp, inp, maxLength)
}
