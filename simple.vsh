uniform mat4 nMatrix;
uniform mat4 mvpMatrix;

attribute vec4 geometry;
attribute vec3 normal;
varying vec4 n;

void main(void)
{
    n = nMatrix * vec4(normal,0);
    gl_Position = mvpMatrix * geometry;
}
