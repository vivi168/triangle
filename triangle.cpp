// gcc triangle.c gl3w.c -I./include -lSDL2 -lGL
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#ifdef _WIN32
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif
#include <GL/gl3w.h>

#include "input_manager.h"
#include "camera.h"
#include "md5.h"

#define FPS 60
#define TICKS_PER_FRAME (1000 / FPS)
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

typedef struct gl_mesh_t {
    int numTris, numVerts;
    GLuint vertex_array_obj, vertex_buffer_obj, element_buffer_obj;
} GlMesh;

typedef struct model_t {
    // char* filename;
    glm::vec3 translate;
    glm::vec3 rotate;
    glm::vec3 scale;
} Model;

SDL_Window* sdl_window;
SDL_GLContext context;
SDL_Event event;
GLuint program = 0;

Uint32 last_time;
Uint32 current_time;

Camera camera;
Model mymodel; // For transformation

MD5Model md5m;
GlMesh mesh; // holds data on GPU


float delta_time;
bool quit = false;
InputManager &input_mgr = InputManager::instance();

glm::mat4 model_mat(Model* m)
{
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, m->translate);
    model = glm::scale(model, m->scale);

    return model;
}

void frame_start()
{
    last_time = current_time;
    current_time = SDL_GetTicks();

    delta_time = (float)(current_time - last_time) / 1000;
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

    if (sdl_window == NULL) {
        fprintf(stderr, "Error while create SDL_Window\n");
        exit(EXIT_FAILURE);
    }

    context = SDL_GL_CreateContext(sdl_window);

    if (gl3wInit()) {
        fprintf(stderr, "failed to init GL3W\n");
        SDL_GL_DeleteContext(context);
        SDL_DestroyWindow(sdl_window);
        SDL_Quit();
        exit(EXIT_FAILURE);
    }

    SDL_SetRelativeMouseMode(SDL_TRUE);
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

    if (!f) {
        // TODO: error handling
        exit(EXIT_FAILURE);
    }

    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    buffer = (char*)malloc(len);

    if (!buffer) {
        // TODO: error handling
        exit(EXIT_FAILURE);
    }

    fread(buffer, 1, len, f);
    fclose(f);

    return buffer;
}

void load_shader()
{
    int success;
    char info_log[1024];
    const char* vert_code = read_shader("vert.glsl");
    const char* frag_code = read_shader("frag.glsl");

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
        fprintf(stderr, "Error: vertex shader: %s\n", info_log);
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
}

typedef enum location_t {
    POSITION_LOC = 0,
    UV_LOC,
    BONE_IDS_LOC,
    WEIGHT_IDS_LOC
} Location;

void init_gl_mesh(GlMesh* mesh, MD5Model* model)
{
    glGenVertexArrays(1, &mesh->vertex_array_obj);
    glGenBuffers(1, &mesh->vertex_buffer_obj);
    glGenBuffers(1, &mesh->element_buffer_obj);

    glBindVertexArray(mesh->vertex_array_obj);

    Vertex* verticesArr = NULL;
    int* indices = NULL;
    prepare_model(&md5m, &verticesArr, &indices, &mesh->numVerts, &mesh->numTris);
    printf("init gl mesh %d %d\n", mesh->numVerts, mesh->numTris);
    
    // vertices
    {
        // vertices = prepare_vertices
        glBindBuffer(GL_ARRAY_BUFFER, mesh->vertex_buffer_obj);
        glBufferData(GL_ARRAY_BUFFER, 
            sizeof(Vertex) * mesh->numVerts,
            &verticesArr[0],
            GL_STATIC_DRAW);
    }

    // triangles
    {
        // triangles = prepare_triangles
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->element_buffer_obj);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * mesh->numTris * 3, &indices[0], GL_STATIC_DRAW);
    }
    

    // vertex position (location 0)
    glVertexAttribPointer(POSITION_LOC, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    glEnableVertexAttribArray(POSITION_LOC);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void destroy_mesh(GlMesh* mesh)
{
    glDeleteVertexArrays(1, &mesh->vertex_array_obj);
    glDeleteBuffers(1, &mesh->vertex_buffer_obj);
    glDeleteBuffers(1, &mesh->element_buffer_obj);
}

void init()
{
    printf("Init\n");

    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
        fprintf(stderr, "Unable to init SDL\n");
        exit(EXIT_FAILURE);
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    create_window();

    glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    load_shader();

    init_gl_mesh(&mesh, &md5m);

    // init model
    {
        mymodel.translate = glm::vec3(0.0f, -3.0f, 0.0f);
        mymodel.scale = glm::vec3(1.0f, 1.0f, 1.0f);
    }
}

void destroy()
{
    destroy_mesh(&mesh);

    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(sdl_window);
    SDL_Quit();
}

void render()
{
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glUseProgram(program);
    glm::mat4 mvp;
    glm::mat4 p = glm::perspective(camera.zoom(), (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 1.0f, 1000.0f);
    glm::mat4 v = camera.look_at();
    glm::mat4 m = model_mat(&mymodel);
    mvp = p * v * m;
    
    GLuint mvp_loc = glGetUniformLocation(program, "mvp");
    glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));

    glBindVertexArray(mesh.vertex_array_obj);
    glDrawElements(GL_TRIANGLES, sizeof(int) * mesh.numTris, GL_UNSIGNED_INT, 0);

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

        render();
        delay();
    }
}

int main(int argc, char **argv)
{
    read_md5model("assets/md5model.bin", &md5m);
    init();

    mainloop();

    destroy();

    // TODO: free model, mesh
    return 0;
}
