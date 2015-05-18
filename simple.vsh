uniform mat4 mvpMatrix;

attribute vec4 geometry;
attribute vec4 normal;

void main(void)
{
    gl_Position = mvpMatrix * geometry;
}
