varying vec4 v_color;
varying vec2 v_texCoord;

uniform int u_hasColors;
uniform int u_hasTexCoords;

uniform sampler2D u_texture;

void main()
{
	if (u_hasColors == 1 && u_hasTexCoords == 0)
	{
		gl_FragColor = v_color;
	}
	if (u_hasColors == 0 && u_hasTexCoords == 1)
	{
		gl_FragColor = texture2D(u_texture, v_texCoord);
	}
	if (u_hasColors == 1 && u_hasTexCoords == 1)
	{
		gl_FragColor = (v_color * texture2D(u_texture, v_texCoord));
	}
}
