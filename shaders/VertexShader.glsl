#version 410 core

layout(location=0) in vec3 inVert;
// layout(location=1) in vec3 inColour;
uniform mat4 MVP;
uniform samplerBuffer colourSampler;

void main()
{
  // read the data from the 1D array using gl_VertexID as the index.
  // float ypos=texelFetch(yPosSampler,gl_VertexID).r;
  // gl_Position = MVP * vec4(xz.x,ypos,xz.y,1);
  gl_Position = MVP * vec4(inVert, 1.0);
  // vertColour = vec3(1.0f, 1.0f, 1.0f);
  vertColour = texelFetch(colourSampler, gl_VertexID).rgb;
}