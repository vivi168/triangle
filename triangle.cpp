#include <iostream>
#include <unordered_map>

#include <cstdlib>
#include <cstdint>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <GL/gl3w.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#ifdef _WIN32
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif

#include "input_manager.h"
#include "camera.h"
#include "md5.h"
#include "obj.h"

#define FPS 60
#define TICKS_PER_FRAME (1000 / FPS)
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

std::unordered_map<std::string, GLuint> loaded_shaders;
GLuint load_shader(std::string);

enum Location {
    POSITION_LOC = 0,
    UV_LOC,
    BLEND_IDX_LOC,
    BLEND_WEIGHTS_LOC
};

struct GlMesh {
    GLuint vertex_buffer_obj, element_buffer_obj;
    GLuint vertex_array_obj;
    GLuint program;

    std::vector<Subset> subsets;
    std::vector<GLuint> textures; // TODO : allow for multiple textures (normal map, etc)

    bool animated;

    void init(const SkinnedMesh& mesh3d)
    {
        int numVerts = mesh3d.vertices.size();
        int numIndices = mesh3d.indices.size();
        subsets = mesh3d.subsets;

        program = loaded_shaders["skinned"];

        textures.resize(subsets.size());
        glGenTextures(subsets.size(), textures.data());

        int i = 0;
        for (const auto& subset : subsets) {
            int width, height, channels, mode;
            std::string filepath = "assets/" + subset.name + ".png"; // HERE: handle multiple textures, with suffix
                                                                     // EG: texture_albedo.png, texture_normal.png etc
            unsigned char* img_data = stbi_load(filepath.c_str(), &width, &height, &channels, 0);

            assert(img_data);

            mode = channels == 4 ? GL_RGBA : GL_RGB;

            glBindTexture(GL_TEXTURE_2D, textures[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, mode, width, height, 0, mode, GL_UNSIGNED_BYTE, img_data);
            glGenerateMipmap(GL_TEXTURE_2D);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            std::cout << filepath  << " " << width << " " << height << " " << textures[i] << "\n";
            std::cout << subset.header.start << " " << subset.header.count << "\n";

            stbi_image_free(img_data);

            i++;
        }

        printf("init skinned gl mesh v %d i %d\n", numVerts, numIndices);

        glGenVertexArrays(1, &vertex_array_obj);
        glBindVertexArray(vertex_array_obj);

        glGenBuffers(1, &vertex_buffer_obj);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_obj);
        glBufferData(GL_ARRAY_BUFFER, sizeof(SkinnedVertex) * numVerts, mesh3d.vertices.data(), GL_STATIC_DRAW);

        glGenBuffers(1, &element_buffer_obj);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_obj);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * numIndices, mesh3d.indices.data(), GL_STATIC_DRAW);

        // vertex position
        glVertexAttribPointer(POSITION_LOC, 3, GL_FLOAT, GL_FALSE, sizeof(SkinnedVertex), (void*)0);
        glEnableVertexAttribArray(POSITION_LOC);

        // texture coordinates
        glVertexAttribPointer(UV_LOC, 2, GL_FLOAT, GL_FALSE, sizeof(SkinnedVertex), (void*)offsetof(SkinnedVertex, uv));
        glEnableVertexAttribArray(UV_LOC);

        // bone idx
        glVertexAttribPointer(BLEND_IDX_LOC, 4, GL_FLOAT, GL_FALSE, sizeof(SkinnedVertex), (void*)offsetof(SkinnedVertex, blend_idx));
        glEnableVertexAttribArray(BLEND_IDX_LOC);

        // bone weigts
        glVertexAttribPointer(BLEND_WEIGHTS_LOC, 4, GL_FLOAT, GL_FALSE, sizeof(SkinnedVertex), (void*)offsetof(SkinnedVertex, blend_weights));
        glEnableVertexAttribArray(BLEND_WEIGHTS_LOC);

        glBindVertexArray(0);

        animated = true;
    }

    void init(std::string filename)
    {
        Mesh mesh3d = Obj::prepare(filename.c_str());

        int numVerts = mesh3d.vertices.size();
        int numIndices = mesh3d.indices.size();
        subsets = mesh3d.subsets;

        printf("init static gl mesh v %d i %d\n", numVerts, numIndices);

        glGenVertexArrays(1, &vertex_array_obj);
        glBindVertexArray(vertex_array_obj);

        glGenBuffers(1, &vertex_buffer_obj);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_obj);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * numVerts, mesh3d.vertices.data(), GL_STATIC_DRAW);

        glGenBuffers(1, &element_buffer_obj);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_obj);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * numIndices, mesh3d.indices.data(), GL_STATIC_DRAW);

        // vertex position
        glVertexAttribPointer(POSITION_LOC, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        glEnableVertexAttribArray(POSITION_LOC);

        // texture coordinates
        glVertexAttribPointer(UV_LOC, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));
        glEnableVertexAttribArray(UV_LOC);

        glBindVertexArray(0);

        animated = false;
    }

    void destroy()
    {
        if (!textures.empty())
            glDeleteTextures(textures.size(), textures.data());

        glDeleteVertexArrays(1, &vertex_array_obj);
        glDeleteBuffers(1, &vertex_buffer_obj);
        glDeleteBuffers(1, &element_buffer_obj);
    }
};

struct Model {
    glm::vec3 translate;
    glm::vec3 rotate;
    glm::vec3 scale;

    glm::mat4 model_mat()
    {
        glm::mat4 mat = glm::mat4(1.0f);
        mat = glm::translate(mat, translate);
        mat = glm::scale(mat, scale);

        return mat;
    }

    void draw(const Camera* camera, const GlMesh* m)
    {
    }
};

SDL_Window* sdl_window;
SDL_GLContext context;
SDL_Event event;

Uint32 last_time;
Uint32 current_time;

Camera camera;
Model mymodel; // For transformation

MD5Model md5m;
MD5Anim md5a;
GlMesh mesh; // holds data on GPU

MD5AnimInfo animinfo;
bool animated = false;

float delta_time;
bool quit = false;
InputManager &input_mgr = InputManager::instance();

void frame_start()
{
    last_time = current_time;
    current_time = SDL_GetTicks();

    delta_time = (float)(current_time - last_time) / 1000;

    {
        float fps = 1.0f / delta_time;
        std::string s = "FPS: " + std::to_string(fps);
        SDL_SetWindowTitle(sdl_window, s.c_str());
    }
}

void delay()
{
    Uint32 frame_time;

    frame_time = SDL_GetTicks() - current_time;
    if (TICKS_PER_FRAME > frame_time) {
        SDL_Delay(TICKS_PER_FRAME - frame_time);
    }
}

void create_window()
{
    printf("Create window\n");

    const char* title = "Sokoban";
    const int width = WINDOW_WIDTH;
    const int height = WINDOW_HEIGHT;

    sdl_window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    assert(sdl_window != NULL);

    context = SDL_GL_CreateContext(sdl_window);

    if (gl3wInit()) {
        fprintf(stderr, "failed to init GL3W\n");
        SDL_GL_DeleteContext(context);
        SDL_DestroyWindow(sdl_window);
        SDL_Quit();
        exit(EXIT_FAILURE);
    }

    SDL_SetRelativeMouseMode(SDL_TRUE);
    SDL_GL_SetSwapInterval(0);
}

const char* read_shader(char const* filename)
{
    char *buffer;
    size_t len;
    FILE* f;

#ifdef _WIN32
    fopen_s(&f, filename, "rb");
#else
    f = fopen(filename, "rb");
#endif

    assert(f);

    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    buffer = (char*)malloc(len + 1);

    assert(buffer);

    fread(buffer, 1, len, f);
    buffer[len] = '\0';
    fclose(f);

    return buffer;
}

GLuint load_shader(std::string name)
{
    GLuint program = 0;
    int success;
    char info_log[1024];

    std::string vert_filename = name + ".vert";
    std::string frag_filename = name + ".frag";

    const char* vert_code = read_shader(vert_filename.c_str());
    const char* frag_code = read_shader(frag_filename.c_str());

    GLuint vert_shader, frag_shader;

    vert_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert_shader, 1, &vert_code, NULL);
    glCompileShader(vert_shader);

    glGetShaderiv(vert_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vert_shader, 1024, NULL, info_log);
        fprintf(stderr, "Error: vertex shader: %s\n", info_log);
    }

    frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag_shader, 1, &frag_code, NULL);
    glCompileShader(frag_shader);

    glGetShaderiv(frag_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(frag_shader, 1024, NULL, info_log);
        fprintf(stderr, "Error: frag shader: %s\n", info_log);
    }

    program = glCreateProgram();
    glAttachShader(program, vert_shader);
    glAttachShader(program, frag_shader);
    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(program, 1024, NULL, info_log);
        fprintf(stderr, "Error: vertex shader: %s\n", info_log);
    }

    glDeleteShader(vert_shader);
    glDeleteShader(frag_shader);

    free((void*)vert_code);
    free((void*)frag_code);

    return program;
}

void init()
{
    printf("Init\n");

    assert(SDL_Init(SDL_INIT_EVERYTHING) == 0);

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    create_window();

    {
        glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE); // enable per subset ?

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    loaded_shaders["skinned"] = load_shader("skinned");

    SkinnedMesh mesh3d = md5m.prepare();
    mesh.init(mesh3d);

    // init model
    {
        mymodel.translate = glm::vec3(0.0f, -3.0f, 0.0f);
        mymodel.scale = glm::vec3(1.0f, 1.0f, 1.0f);
    }
}

void destroy()
{
    mesh.destroy();

    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(sdl_window);
    SDL_Quit();
}

void render()
{
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glUseProgram(mesh.program);
    
    {
        glm::mat4 mvp;

        glm::mat4 p = glm::perspective(camera.zoom(), (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 1.0f, 1000.0f);
        glm::mat4 v = camera.look_at();
        glm::mat4 m = mymodel.model_mat();
        mvp = p * v * m;

        GLuint mvp_loc = glGetUniformLocation(mesh.program, "mvp");
        glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));
    }

    {
        // TODO : if not animated, push identity matrix
        std::vector<glm::mat4> inv = md5m.inv_bindpose_matrices(); // TODO compute only once per model
        std::vector<glm::mat4> bones = md5a.bone_matrices(animinfo.currFrame);

        for (int i = 0; i < md5a.header.numJoints; i++) {
            char loc[10];
#ifdef _WIN32
            sprintf_s(loc, 10, "bones[%d]", i);
#else
            sprintf(loc, "bones[%d]", i);
#endif
            GLuint bones_loc = glGetUniformLocation(mesh.program, loc);
            glm::mat4 final_bones = bones[i] * inv[i];
            glUniformMatrix4fv(bones_loc, 1, GL_FALSE, glm::value_ptr(final_bones));
        }
    }

    glBindVertexArray(mesh.vertex_array_obj);

    int i = 0;
    for (const auto& subset : mesh.subsets) {
        glActiveTexture(GL_TEXTURE0);
        glUniform1i(glGetUniformLocation(mesh.program, "texture_sampler"), 0);
        glBindTexture(GL_TEXTURE_2D, mesh.textures[i++]);

        glDrawElements(GL_TRIANGLES, subset.header.count, GL_UNSIGNED_INT, (void*)(sizeof(int) * subset.header.start));
    }

    SDL_GL_SwapWindow(sdl_window);
}

void process_input()
{
    camera.process_mouse(input_mgr.mouse_x(), input_mgr.mouse_y());

    if (input_mgr.quit_requested() || input_mgr.is_pressed(SDLK_ESCAPE)) {
        quit = true;
    }

    if (input_mgr.is_held(SDLK_w)) {
        camera.process_keyboard(CameraDirection::FORWARD, delta_time);
    }

    if (input_mgr.is_held(SDLK_s)) {
        camera.process_keyboard(CameraDirection::BACKWARD, delta_time);
    }

    if (input_mgr.is_held(SDLK_a)) {
        camera.process_keyboard(CameraDirection::LEFTWARD, delta_time);
    }

    if (input_mgr.is_held(SDLK_d)) {
        camera.process_keyboard(CameraDirection::RIGHTWARD, delta_time);
    }

    if (input_mgr.is_held(SDLK_UP)) {
        camera.process_keyboard(CameraDirection::UP, delta_time);
    }

    if (input_mgr.is_held(SDLK_DOWN)) {
        camera.process_keyboard(CameraDirection::DOWN, delta_time);
    }

    if (input_mgr.is_held(SDLK_LEFT)) {
        camera.process_keyboard(CameraDirection::LEFT, delta_time);
    }

    if (input_mgr.is_held(SDLK_RIGHT)) {
        camera.process_keyboard(CameraDirection::RIGHT, delta_time);
    }
}


void mainloop()
{
    last_time = SDL_GetTicks();
    current_time = SDL_GetTicks();
    delta_time = 0.0f;

    while (!quit) {
        frame_start();

        input_mgr.update();
        process_input();

        if (animated) {
            animate(&md5a, &animinfo, delta_time);
            // TODO interpolate skeleton
        }

        render();
        //delay();
    }
}

int main(int argc, char **argv)
{
    md5m.read("assets/md5model.bin");
    md5a.read("assets/md5anim.bin");

    Obj::prepare("assets/collision/cube.bin");
    
    {
        // TODO -> have anim info as part of anim ?
        animinfo.currFrame = 0;
        animinfo.nextFrame = 1;

        animinfo.time = 0;
        animinfo.frameDuration = 1.0 / md5a.header.frameRate;
        animated = true;
    }

    init();

    mainloop();

    destroy();

    // TODO: free model, mesh
    return 0;
}
