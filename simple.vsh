uniform mat4 mvpMatrix;

attribute vec4 geometry;

void main(void)
{
    gl_Position = mvpMatrix * geometry;
}
