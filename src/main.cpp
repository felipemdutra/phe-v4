#include <chrono>
#include <glm/ext/vector_float3.hpp>
#include <wrapgl/wrapgl.h>
#include <glm/glm.hpp>

#include <string>
#include <memory>

#include <GLFW/glfw3.h>

#include "../include/pendulum.h"
#include "../include/soft_body.h"
#include "../include/util/util.h"
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

        camera.SetPosition(vec3(0.0f, 0.0f, 40.0f));

        Pendulum p1(vec3(-5.0f, 0.0f, 0.0f), 3, 3.0f, true);
        Pendulum p2(vec3(5.0f, 0.0f, 0.0f), 3, 3.0f, true);

        RigidBody b(Shape::kPyramid, 5.0f, vec3(1.0f, 1.0f, 1.0f), false);

        bool primeiro = true;
        int frames = 0;

        //SoftBody sb(vec3(0.0f, 0.0f, 0.0f), 0.1f, 1.0);

        while (!window.ShouldClose()) {
                auto curr_t = std::chrono::high_resolution_clock::now();
                dt = std::chrono::duration<float>(curr_t - prev_t).count();
                prev_t = curr_t;

                renderer.Clear(0.1f, 0.1f, 0.1f, 1.0f, true);
                camera.Update();

                for (int i = 0; i < 3; i++) {
                        p1.GetBob(i)->IntegrateLinearAcceleration(vec3(0.0f, -kGravity, 0.0f), dt);
                        p2.GetBob(i)->IntegrateLinearAcceleration(vec3(0.0f, -kGravity, 0.0f), dt);

                        if (primeiro == true) {
                                frames += 1;
                                if (frames == 1000) {
                                        p1.GetBob(2)->IntegrateLinearImpulse(vec3(130.0f, 0.0f, 0.0f));
                                        p2.GetBob(2)->IntegrateLinearImpulse(vec3(130.0f, 0.0f, 0.0f));

                                        p1.GetBob(1)->IntegrateLinearImpulse(vec3(-90.0f, 0.0f, 0.0f));
                                        p2.GetBob(1)->IntegrateLinearImpulse(vec3(-90.0f, 0.0f, 0.0f));
                                        primeiro = false;
                                }
                        }
                }

                p1.Update(dt);
                p2.Update(dt);

                p1.Draw(renderer);
                p2.Draw(renderer);

                window.SwapBuffers();

                // Just so the OS doesn't think we crashed.
                glfwPollEvents();
        }

        return 0;
}

