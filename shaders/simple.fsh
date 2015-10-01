uniform vec4 color;

uniform vec4 ambient_light;
uniform vec4 diffuse_light;

uniform vec4 direction_light;

varying vec4 n;

void main (void)
{
    gl_FragColor = (ambient_light * color) + (diffuse_light * color * dot(n, direction_light));
}
