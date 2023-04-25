#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 4) out;

in VS_OUT {
    vec3 color;
} gs_in[];

out vec3 fColor;

/*
Shape of boid
        *
       *  *
      *    *
     *      *
    *        *
   *    *     *  <----- New point
  *  **   **   *
 * *          * *

*/
void build_boid(vec4 position0,vec4 position1,vec4 position2)
{    
    fColor = gs_in[0].color; // gs_in[0] since there's one colour
    // Barycenter
    vec4 Barycenter = (position0+position1+position2) / 3.;
    // First triangle    
    gl_Position = position0; // 1:bottom-left   
    EmitVertex();   
    gl_Position = position1; // 2:bottom-right
    EmitVertex();
    gl_Position = Barycenter; // 3:top-left
    EmitVertex();
    EndPrimitive();

    // Second triangle
    gl_Position = position0; // 1:bottom-left   
    EmitVertex();   
    gl_Position = position2; // 2:bottom-right
    EmitVertex();
    gl_Position = Barycenter; // 3:top-left
    EmitVertex();
    EndPrimitive();
}

void main() {    
    build_boid(gl_in[0].gl_Position,gl_in[1].gl_Position,gl_in[2].gl_Position);
}