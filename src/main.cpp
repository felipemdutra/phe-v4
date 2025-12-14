#include <chrono>
#include <cmath>
#include <glm/ext/vector_float3.hpp>
#include <wrapgl/wrapgl.h>
#include <glm/glm.hpp>

#include <string>
#include <memory>

#include <GLFW/glfw3.h>

#include "../include/soft_body.h"
#include "../include/rigid_body.h"
#include "../include/util/util.h"

constexpr float kInitialWindowWidth  = 800.0f;
constexpr float kInitialWindowHeight = 600.0f;

const std::string kTitle = "PHE-v4";

const std::string kDefaultProgramName = "default";

double dt = 0.0f;
auto prev_t = std::chrono::high_resolution_clock::now();

int main(void)
{
        using namespace glm;

        auto window = wgl::Window(kInitialWindowWidth, kInitialWindowHeight, kTitle);

        auto renderer = wgl::Renderer();

        auto program = std::make_shared<wgl::ShaderProgram>(
                        "./shaders/default-vertex.glsl", 
                        "./shaders/default-fragment.glsl"
                        );

        // To use the program, we need to bind it to the renderer.
        renderer.BindProgram(kDefaultProgramName, program.get());
        renderer.UseProgram(kDefaultProgramName);

        auto camera = wgl::PerspectiveCamera(
                        program.get(),
                        "view",
                        "proj",
                        45.0f,
                        (float)window.GetWidth() / window.GetHeight(),
                        0.1f,
                        1000.0f);

        camera.SetPosition(vec3(0.0f, 0.0f, 20.0f));

        //float pm = 5.0f;
        //float k  = 0.3f;

        //SoftBody sb(vec3(0.0f, 3.0f, -20.0f), pm, k);

        RigidBody rb = RigidBody(Shape::kCube, 1.0f, vec3(1.0f, 1.0f, 1.0f), false);

        int frames = 0;
        bool first = true;

        while (!window.ShouldClose()) {
                auto curr_t = std::chrono::high_resolution_clock::now();
                dt = std::chrono::duration<float>(curr_t - prev_t).count();
                prev_t = curr_t;

                renderer.Clear(0.1f, 0.1f, 0.1f, 1.0f, true);
                camera.Update();

                if (first) {
                        rb.IntegrateAccelerations(vec3(10.0f, 40.0f, -60.0f), vec3(0.1f, 0.0f, 0.0f), dt);
                }

                rb.IntegrateVelocities(dt);

                rb.Draw(renderer);

                window.SwapBuffers();

                // Just so the OS doesn't think we crashed.
                glfwPollEvents();
        }

        return 0;
}

