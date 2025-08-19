#include <chrono>
#include <wrapgl/wrapgl.h>
#include <glm/glm.hpp>

#include <string>
#include <memory>

#include <GLFW/glfw3.h>

#include "../include/rigid_body.h"
#include "../include/math/constants.h"

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

        camera.SetPosition(vec3(0.0f, 2.5f, 8.0f));
        camera.SetRotation(vec3(/*pitch*/-20.0f, /*yaw*/-90.0f, 0.0f));

        RigidBody body = RigidBody(Shape::kPyramid, 5.0f, vec3(1.0f, 1.0f, 1.0f));
        //body.IntegrateLinearVelocity(glm::vec3(0.0f, 20.0f, -60.0f, dt);

        bool is_first = true;

        while (!window.ShouldClose()) {
                auto curr_t = std::chrono::high_resolution_clock::now();
                dt = std::chrono::duration<float>(curr_t - prev_t).count();
                prev_t = curr_t;

                renderer.Clear(0.1, 0.1, 0.1, 1.0f, true);
                camera.Update();

                // Apply gravity.
                body.IntegrateLinearAcceleration(glm::vec3(0.0f, -kGravity * 2.0, 0.0f), dt);

                if (is_first) {
                        body.IntegrateImpulses(glm::vec3(10.0f, 20.0f, -60.0f), glm::vec3(0.080f, 0.030f, 0.080f), dt);
                        is_first = false;
                }

                body.IntegrateVelocities(dt);

                body.Draw(renderer);

                window.SwapBuffers();

                // Just so the OS doesn't think we crashed.
                glfwPollEvents();
        }

        return 0;
}

