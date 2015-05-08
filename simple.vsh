attribute vec4 a_position;
attribute vec4 a_color;
attribute vec2 a_texCoord;
attribute vec3 a_normal;

uniform mat4 u_modelViewMatrix;
uniform mat4 u_modelViewProjMatrix;

varying vec4 v_color;
varying vec2 v_texCoord;

struct LightSource
{
         vec3 position;
         vec4 ambientColor;
         vec4 diffuseColor;
         vec4 specularColor;
         int  lightType;
};

const int POINT_LIGHT 		= 0;
const int DIRECTIONAL_LIGHT 	= 1;
const int SPOT_LIGHT		= 2;

uniform LightSource[10] u_lights;

struct Material
{
         vec4 ambientReflection;
         vec4 diffuseReflection;
         vec4 specularReflection;
         float shininess;
};

uniform Material u_mat;

void main()
{
         vec3 modelViewVertex = vec3(u_modelViewMatrix * a_position);
         vec3 modelViewNormal = vec3(u_modelViewMatrix * vec4(a_normal, 0.0));

         vec4 lightFactor = vec4(0.0, 0.0, 0.0, 0.0);
         for (int i = 0; i < u_lights.length(); i++)
         {
                  float attenuation;
                  float diffuseFactor;
                  if (u_lights[i].lightType == DIRECTIONAL_LIGHT)
                  {
                           attenuation = 1.0;
                  }
                  else
                  {
                           float distance  = length(u_lights[i].position - modelViewVertex);
                           attenuation = (1.0 / distance);
                  }

                  vec3 lightVector = normalize(u_lights[i].position - modelViewVertex);
                  diffuseFactor = max(dot(lightVector, modelViewNormal), 0.0);
                  lightFactor += u_lights[i].ambientColor * u_mat.ambientReflection + attenuation * (u_lights[i].diffuseColor * u_mat.diffuseReflection * diffuseFactor);
         }
         v_color = lightFactor * a_color;

         v_texCoord = a_texCoord;

         gl_Position = u_modelViewProjMatrix * a_position;
}
