// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "PointCloudRenderer.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <algorithm>
#include <array>
#include <thread>

#include "PointCloudShaders.h"
#include "ViewControl.h"
#include "Helpers.h"

using namespace linmath;
using namespace Visualization;

float offset2[] = { 0.0, 0.0, 0.0 };
float offset3[] = { 0.0, 0.0, 0.0 };
float offset4[] = { 0.0, 0.0, 0.0 };
float offset5[] = { 0.0, 0.0, 0.0 };
float offset6[] = { 0.0, 0.0, 0.0 };
float offset7[] = { 0.0, 0.0, 0.0 };
float offset8[] = { 0.0, 0.0, 0.0 };

PointCloudRenderer::PointCloudRenderer()
{
    mat4x4_identity(m_view);
    mat4x4_identity(m_projection);
}

PointCloudRenderer::~PointCloudRenderer()
{
    Delete();
}

void PointCloudRenderer::Create(GLFWwindow* window)
{
    CheckAssert(!m_initialized);
    m_initialized = true;

    m_window = window;
    glfwMakeContextCurrent(window);

    // Context Settings
    glEnable(GL_PROGRAM_POINT_SIZE);

    m_vertexShader = glCreateShader(GL_VERTEX_SHADER);
    const GLchar* vertexShaderSources[] = { glslShaderVersion, glslPointCloudVertexShader };
    int numVertexShaderSources = sizeof(vertexShaderSources) / sizeof(*vertexShaderSources);
    glShaderSource(m_vertexShader, numVertexShaderSources, vertexShaderSources, NULL);
    glCompileShader(m_vertexShader);
    ValidateShader(m_vertexShader);

    m_fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    const GLchar* fragmentShaderSources[] = { glslShaderVersion, glslPointCloudFragmentShader};
    int numFragmentShaderSources = sizeof(fragmentShaderSources) / sizeof(*fragmentShaderSources);
    glShaderSource(m_fragmentShader, numFragmentShaderSources, fragmentShaderSources, NULL);
    glCompileShader(m_fragmentShader);
    ValidateShader(m_fragmentShader);

    m_shaderProgram = glCreateProgram();
    glAttachShader(m_shaderProgram, m_vertexShader);
    glAttachShader(m_shaderProgram, m_fragmentShader);
    glLinkProgram(m_shaderProgram);
    ValidateProgram(m_shaderProgram);

    glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);
    glGenBuffers(1, &m_vertexBufferObject);
    glGenBuffers(1, &m_quadElementBufferObject);
    m_viewIndex = glGetUniformLocation(m_shaderProgram, "view");
    m_projectionIndex = glGetUniformLocation(m_shaderProgram, "projection");
    m_enableShadingIndex = glGetUniformLocation(m_shaderProgram, "enableShading");
    //m_xyTableSamplerIndex = glGetUniformLocation(m_shaderProgram, "xyTable");
    //m_depthSamplerIndex = glGetUniformLocation(m_shaderProgram, "depth");
    m_colorSamplerIndex = glGetUniformLocation(m_shaderProgram, "colorTexture");

    InitializeTexture();

}

void PointCloudRenderer::Delete()
{
    if (!m_initialized)
    {
        return;
    }

    m_initialized = false;
    glDeleteBuffers(1, &m_vertexBufferObject);
    glDeleteBuffers(1, &m_quadElementBufferObject);

    glDeleteShader(m_vertexShader);
    glDeleteShader(m_fragmentShader);
    glDeleteProgram(m_shaderProgram);
}

void PointCloudRenderer::InitializeTexture()
{
    //Create buffer object
    glGenBuffers(1, &m_colorTextureObject);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, m_colorTextureObject);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, 640 * 4608 * 4, NULL, GL_STREAM_DRAW);

}

void PointCloudRenderer::UpdatePointClouds(
    GLFWwindow* window,
    const PointCloudVertex* point3ds,
    uint32_t numPoints,
    uint32_t width, uint32_t height,
    bool useTestPointClouds)
{
    if (window != m_window)
    {
        Create(window);
    }


    //Mapping texture
   
    /*
    void* mappedBuffer = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
    memcpy(mappedBuffer, colorFrame, 640 * 4608 * 4);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, m_colorTextureObject);
    glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 640, 4608, 0, GL_BGRA, GL_UNSIGNED_BYTE, 0);
    */

    //glBindVertexArray(m_vertexArrayObject);
    //// Create buffers and bind the geometry
    //glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
    //glBufferData(GL_ARRAY_BUFFER, numPoints * sizeof(PointCloudVertex), point3ds, GL_DYNAMIC_DRAW);
    //
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_quadElementBufferObject);
    //glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndices * sizeof(uint32_t), Indices, GL_DYNAMIC_DRAW);
    //
    ///*
    //for (int i = 0; i < numIndices; i++) {
    //    printf("%d \n",Indices[i]);
    //}
    //*/
    //
    //glEnableVertexAttribArray(0);
    //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PointCloudVertex), (void*)0);
    //
    //glEnableVertexAttribArray(3);
    //glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(PointCloudVertex), (void*)offsetof(PointCloudVertex, UV));
    //
    //
    //glBindVertexArray(0);
    //
    ////m_drawArraySize = useTestPointClouds ? 8 : GLsizei(numPoints);
    //m_drawArraySize = useTestPointClouds ? 8 : GLsizei(numIndices);

    glBindVertexArray(m_vertexArrayObject);
    // Create buffers and bind the geometry
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
    glBufferData(GL_ARRAY_BUFFER, numPoints * sizeof(PointCloudVertex), point3ds, GL_DYNAMIC_DRAW);

    // Set the vertex attribute pointers
    // Vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PointCloudVertex), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(PointCloudVertex), (void*)offsetof(PointCloudVertex, Color));


    glBindVertexArray(0);

    m_drawArraySize = useTestPointClouds ? 8 : GLsizei(numPoints);
}

void PointCloudRenderer::SetShading(bool enableShading)
{
    m_enableShading = enableShading;
}

void PointCloudRenderer::Render()
{
    std::array<int, 4> data; // x, y, width, height

    glGetIntegerv(GL_VIEWPORT, data.data());
    Render(data[2], data[3]);
}

void PointCloudRenderer::Render(int width, int height)
{
    glEnable(GL_DEPTH_TEST);
    // Enable blending
   
    /*
    glEnable(GL_LINE_SMOOTH);    
    glEnable(GL_POLYGON_SMOOTH);
    */

    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /*
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    glFrontFace(GL_CCW);
    */

    glDisable(GL_STENCIL_TEST);   
    glDisable(GL_BLEND);    
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    //glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    

    //float pointSize;
    //if (m_pointCloudSize)
    //{
    //    pointSize = m_pointCloudSize.value();
    //} 
    //else if (m_width == 0 || m_height == 0)
    //{
    //    pointSize = m_defaultPointCloudSize;
    //}
    //else
    //{
    //    pointSize = std::min(2.f * width / (float)m_width, 2.f * height / (float)m_height)*0.5;
    //}
    //glPointSize(pointSize);
    //
    //glUseProgram(m_shaderProgram);
    //
    //// Update model/view/projective matrices in shader
    //glUniformMatrix4fv(m_viewIndex, 1, GL_FALSE, (const GLfloat*)m_view);
    //glUniformMatrix4fv(m_projectionIndex, 1, GL_FALSE, (const GLfloat*)m_projection);
    //
    //// Render point cloud
    //glBindVertexArray(m_vertexArrayObject);
    ////glDrawArrays(GL_POINTS, 0, m_drawArraySize);
    //
    //glDrawElements(GL_POINTS, m_drawArraySize, GL_UNSIGNED_INT, 0);
    ////glDrawElements(GL_QUADS, m_drawArraySize, GL_UNSIGNED_INT, 0);
    ////glDrawArrays(GL_QUADS, 0, m_drawArraySize);
    //glBindVertexArray(0);
    //
    ////glfwSwapBuffers(m_window);


    float pointSize;
    if (m_pointCloudSize)
    {
        pointSize = m_pointCloudSize.value();
    }
    else if (m_width == 0 || m_height == 0)
    {
        pointSize = m_defaultPointCloudSize;
    }
    else
    {
        pointSize = std::min(2.f * width / (float)m_width, 2.f * height / (float)m_height) * 0.5;
    }
    //pointSize = 2.0f;
    glPointSize(pointSize);

    glUseProgram(m_shaderProgram);

    // Update model/view/projective matrices in shader
    glUniformMatrix4fv(m_viewIndex, 1, GL_FALSE, (const GLfloat*)m_view);
    glUniformMatrix4fv(m_projectionIndex, 1, GL_FALSE, (const GLfloat*)m_projection);

    // Render point cloud
    glBindVertexArray(m_vertexArrayObject);
    glDrawArrays(GL_POINTS, 0, m_drawArraySize);
    glBindVertexArray(0);
}

void PointCloudRenderer::ChangePointCloudSize(float pointCloudSize)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_pointCloudSize = pointCloudSize;
}

/*
void PointCloudRenderer::render_slider()
{
    static const int flags = ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize;

    ImGui::SetNextWindowPos({ 0, 0 });
    ImGui::SetNextWindowSize({ 300, 200 });

    ImGui::Begin("slider", nullptr, flags);

    ImGui::SliderFloat3(" Camera 02", offset2, -0.1, 0.1);
    ImGui::SliderFloat3(" Camera 03", offset3, -0.1, 0.1);
    ImGui::SliderFloat3(" Camera 04", offset4, -0.1, 0.1);
    ImGui::SliderFloat3(" Camera 05", offset5, -0.1, 0.1);
    ImGui::SliderFloat3(" Camera 06", offset6, -0.1, 0.1);
    ImGui::SliderFloat3(" Camera 07", offset7, -0.1, 0.1);
    ImGui::SliderFloat3(" Camera 08", offset8, -0.1, 0.1);



    //draw_text(140, 350, yaw_value);


    //ImGui::Button("Save");

    if (ImGui::Button("Save"))
    {
        // do something
        printf("Saved!!\n");
        printf("%f %f %f\n", offset2[0], offset2[1], offset2[2]);
        printf("%f %f %f\n", offset3[0], offset3[1], offset3[2]);
        printf("%f %f %f\n", offset4[0], offset4[1], offset4[2]);
        printf("%f %f %f\n", offset5[0], offset5[1], offset5[2]);
        printf("%f %f %f\n", offset6[0], offset6[1], offset6[2]);
        printf("%f %f %f\n", offset7[0], offset7[1], offset7[2]);
        printf("%f %f %f\n", offset8[0], offset8[1], offset8[2]);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
    {
        // do something
        printf("Reset!!\n");
        offset2[0] = 0.0f, offset2[1] = 0.0f, offset2[2] = 0.0f;
        offset3[0] = 0.0f, offset3[1] = 0.0f, offset3[2] = 0.0f;
        offset4[0] = 0.0f, offset4[1] = 0.0f, offset4[2] = 0.0f;
        offset5[0] = 0.0f, offset5[1] = 0.0f, offset5[2] = 0.0f;
        offset6[0] = 0.0f, offset6[1] = 0.0f, offset6[2] = 0.0f;
        offset7[0] = 0.0f, offset7[1] = 0.0f, offset7[2] = 0.0f;
        offset8[0] = 0.0f, offset8[1] = 0.0f, offset8[2] = 0.0f;
    }

    /*
    if (ImGui::IsAnyItemActive()) {
        app_state.over = 1;
    }
    else {
        app_state.over = 0;
    }
    */
    //ImGui::End();
//}
