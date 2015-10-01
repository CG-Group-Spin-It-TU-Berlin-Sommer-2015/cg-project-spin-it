attribute highp vec4 geometry;
attribute lowp vec4 color;
varying lowp vec4 col;
uniform highp mat4 mvpMatrix;

void main() {
   col = color;
   gl_Position = mvpMatrix * geometry;
}
