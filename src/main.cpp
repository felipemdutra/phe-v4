#include <wrapgl/wrapgl.h>
#include <glm/glm.hpp>

#include <string>
#include <memory>

#include <GLFW/glfw3.h>

constexpr float kInitialWindowWidth  = 800.0f;
constexpr float kInitialWindowHeight = 600.0f;

const std::string kTitle = "PHE-v4";

const std::string kDefaultProgramName = "default";

int main(void)
{
        using namespace glm;

        auto window = wgl::Window(kInitialWindowWidth, kInitialWindowHeight, kTitle);

        auto renderer = wgl::Renderer();

        auto program = std::make_shared<wgl::ShaderProgram>("./shaders/default-vertex.glsl", "./shaders/default-fragment.glsl");

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

        // Simple mesh just to test everything.
        wgl::VertexLayout layout;
        layout.AddAttr(0, wgl::AttributeType::kPosition, 3, GL_FLOAT, false);
        layout.AddAttr(1, wgl::AttributeType::kColor, 3, GL_FLOAT, true);

        vec3 rgb = vec3(1.0f, 0.2f, 0.3f);
        wgl::Mesh *mesh = wgl::MeshFactory::GetCube(layout, &rgb);
        
        renderer.SetUniformMatrix4f("model", mat4(1.0f));

        while (!window.ShouldClose()) {
                renderer.Clear(0.1, 0.1, 0.1, 1.0f, true);
                camera.Update();

                mesh->Draw(renderer);

                window.SwapBuffers();

                // Just so the OS doesn't think we crashed.
                glfwPollEvents();
        }

        wgl::MeshFactory::DestroyMesh(mesh);

        return 0;
}

