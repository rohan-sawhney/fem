#include "ModelProblem.h"
#include "RenderData.h"
#include "Camera.h"

#define ESCAPE 27
#define DIGIT_OFFSET 48
#define MAX_PICKED_ID 16777215

int gridX = 600;
int gridY = 600;

GLuint transformUbo;
GLuint lightUbo;

string path;
string shaderPath;
Shader meshShader;
Shader normalShader;
Shader wireframeShader;
Shader pickShader;

Camera camera;
float lastTime = 0.0;
float dt = 0.0;
float lastX = 0.0, lastY = 0.0;
float pressX = 0.0, pressY = 0.0;
bool keys[256];
bool firstMouse = true;

Mesh mesh;
GLMesh glMesh(mesh);

const Vector3f defaultColor(0.0, 0.0, 1.0);
vector<Vector3f> colors;

const Vector3f lightPosition(0.0, 3.0, -3.0);
const Vector3f lightColor(1.0, 1.0, 1.0);

VectorXd u;
Vector3d up(0.0, 1.0, 0.0);

bool success = true;
bool showNormals = false;
bool showWireframe = false;
bool pickingEnabled = false;

int bdyType(int index)
{
    const Edge& e = mesh.edges[index];
    Vector3d v = e.he->flip->vertex->position - e.he->vertex->position; 
    if (fabs(v.dot(up)) < 1e-3) {
        return DIRICHLET;
    }
    
    return NEUMANN;
}

void fRhs(const Vector2d& x, double& f)
{
    f = 0.0;
}

void beta(const Vector2d& x, double& b)
{
    b = 0.0;
}

void alpha(const Vector2d& x, Matrix2d& a)
{
    a.setIdentity();
}

void exact(const Vector2d& x, double& u)
{
    u = cos(M_PI*x(0))*exp(M_PI*x(1));
}

void dExact(const Vector2d& x, Vector2d& dudx)
{
    dudx(0) = -M_PI*sin(M_PI*x(0))*exp(M_PI*x(1));
    dudx(1) =  M_PI*cos(M_PI*x(0))*exp(M_PI*x(1));
}

void setupShaders()
{
    meshShader.setup(shaderPath, "Model.vert", "", "Model.frag");
    normalShader.setup(shaderPath, "Normal.vert", "Normal.geom", "Normal.frag");
    wireframeShader.setup(shaderPath, "Wireframe.vert", "", "Wireframe.frag");
    pickShader.setup(shaderPath, "Flat.vert", "", "Flat.frag");
}

void setupUniformBlocks()
{
    // 1) generate transform indices
    GLuint meshShaderIndex = glGetUniformBlockIndex(meshShader.program, "Transform");
    GLuint normalShaderIndex = glGetUniformBlockIndex(normalShader.program, "Transform");
    GLuint wireframeShaderIndex = glGetUniformBlockIndex(wireframeShader.program, "Transform");
    GLuint pickShaderIndex = glGetUniformBlockIndex(pickShader.program, "Transform");
    
    // bind
    glUniformBlockBinding(meshShader.program, meshShaderIndex, 0);
    glUniformBlockBinding(normalShader.program, normalShaderIndex, 0);
    glUniformBlockBinding(wireframeShader.program, wireframeShaderIndex, 0);
    glUniformBlockBinding(pickShader.program, pickShaderIndex, 0);
    
    // add transform data
    glGenBuffers(1, &transformUbo);
    glBindBuffer(GL_UNIFORM_BUFFER, transformUbo);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, transformUbo);
    glBufferData(GL_UNIFORM_BUFFER, 3*sizeof(Matrix4f), NULL, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    
    // 2) generate light index
    meshShaderIndex = glGetUniformBlockIndex(meshShader.program, "Light");
    
    // bind
    glUniformBlockBinding(meshShader.program, meshShaderIndex, 1);
    
    // add light data
    glGenBuffers(1, &lightUbo);
    glBindBuffer(GL_UNIFORM_BUFFER, lightUbo);
    glBufferData(GL_UNIFORM_BUFFER, 2*sizeof(Vector4f), NULL, GL_STATIC_DRAW); // std140 alignment
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, lightUbo);
    
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(Vector4f), lightPosition.data());
    glBufferSubData(GL_UNIFORM_BUFFER, sizeof(Vector4f), sizeof(Vector4f), lightColor.data());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void printInstructions()
{
    cerr << "' ': solve\n"
         << "1: toggle normals\n"
         << "2: toggle wireframe\n"
         << "w/s: move in/out\n"
         << "a/d: move left/right\n"
         << "e/q: move up/down\n"
         << "escape: exit program\n"
         << endl;
}

void setDefaultColors()
{
    colors.resize(mesh.vertices.size());
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        colors[v->index] = defaultColor;
    }
}

void visualizeFem()
{
    double min = u.minCoeff();
    double range = u.maxCoeff() - min;
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        double scale = (u(v->index) - min)/range;
        if (scale > 0.5) colors[v->index] = Vector3f(0.0, 0.0, scale - 0.5);
        else colors[v->index] = Vector3f(0.5 - scale, 0.0, 0.0);
    }
    glMesh.update(colors);
}

void init()
{
    // enable depth testing and multisampling
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    
    // setup shaders and blocks
    setupShaders();
    setupUniformBlocks();
    
    // read mesh
    success = mesh.read(path);
    if (success) {
        setDefaultColors();
        glMesh.setup(colors);
    }
    
    // print instructions
    printInstructions();
}

void solveFem()
{
    Solution solution;
    solution.exact = exact;
    solution.dExact = dExact;
    
    ModelData modelData;
    modelData.bdyType = bdyType;
    modelData.fRhs = fRhs;
    modelData.beta = beta;
    modelData.alpha = alpha;
    
    ModelProblem model(&mesh);
    u = model.solve(QUADRATIC, &solution, &modelData);
}

void uploadCameraTransforms()
{
    // set camera transformations
    glBindBuffer(GL_UNIFORM_BUFFER, transformUbo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4),
                    glm::value_ptr(camera.projectionMatrix(gridX, gridY)));
    glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4),
                    glm::value_ptr(camera.viewMatrix()));
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    
    // set view position
    meshShader.use();
    glUniform3f(glGetUniformLocation(meshShader.program, "viewPosition"),
                camera.pos.x(), camera.pos.y(), camera.pos.z());
}

void uploadMeshTransform(const Matrix4f& transform)
{
    // set transform
    glBindBuffer(GL_UNIFORM_BUFFER, transformUbo);
    glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(Matrix4f), sizeof(Matrix4f),
                    transform.data());
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void processPickedElement(int id)
{
    int f = (int)mesh.faces.size()-1;
    int fv = f + (int)mesh.vertices.size();
    if (id < f) {
        // Do something with the picked face

    } else if (id < fv) {
        // Do something with the picked vertex
    }
}

void pick()
{
    if (pickingEnabled) {
        // clear
        glClearColor(1.0, 1.0, 1.0, 1.0);
        glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDepthFunc(GL_LESS);
        
        // draw pick
        Matrix4f transform = Matrix4f::Identity();
        uploadMeshTransform(transform);
        glMesh.drawPick(pickShader);
        
        glFlush();
        glFinish();
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        
        unsigned char data[4];
        glReadPixels(pressX, gridY - pressY, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);
        
        // Convert color to ID
        int pickedId = data[0] + data[1]*256 + data[2]*256*256;
        if (pickedId != MAX_PICKED_ID) processPickedElement(pickedId);
        
        pickingEnabled = false;
    }
}

void draw(GLMesh& m)
{
    m.draw(meshShader);
    if (showNormals) m.draw(normalShader);
    if (showWireframe) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        m.draw(wireframeShader);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
}

void display()
{
    float elapsedTime = glutGet(GLUT_ELAPSED_TIME);
    dt = (elapsedTime - lastTime) / 1000.0;
    lastTime = elapsedTime;
    
    if (success) {
        // upload camera transforms
        uploadCameraTransforms();
        
        // pick
        pick();
        
        // clear
        glClearColor(0.1, 0.1, 0.1, 1.0);
        glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDepthFunc(GL_LEQUAL);
        
        // draw
        Matrix4f transform = Matrix4f::Identity();
        uploadMeshTransform(transform);
        draw(glMesh);
        
        // swap
        glutSwapBuffers();
    }
}

void idle()
{
    glutPostRedisplay();
}

void reset()
{
    glMesh.reset();
    meshShader.reset();
    normalShader.reset();
    wireframeShader.reset();
    pickShader.reset();
    glDeleteBuffers(1, &transformUbo);
    glDeleteBuffers(1, &lightUbo);
}

void keyboardPressed(unsigned char key, int x, int y)
{
    keys[key] = true;
    
    if (keys[ESCAPE]) {
        reset();
        exit(0);
    
    } else if (keys[' ']) {
        clock_t t = clock();
        solveFem();
        cout << "Time: " << (double)(clock() - t)/CLOCKS_PER_SEC << "s" << endl;
        visualizeFem();
        
    } else if (keys[DIGIT_OFFSET + 1]) {
        showNormals = !showNormals;
        
    } else if (keys[DIGIT_OFFSET + 2]) {
        showWireframe = !showWireframe;
    
    } else if (keys['a']) {
        camera.processKeyboard(LEFT, dt);
        
    } else if (keys['d']) {
        camera.processKeyboard(RIGHT, dt);
        
    } else if (keys['w']) {
        camera.processKeyboard(FORWARD, dt);
        
    } else if (keys['s']) {
        camera.processKeyboard(BACKWARD, dt);
        
    } else if (keys['e']) {
        camera.processKeyboard(UP, dt);
        
    } else if (keys['q']) {
        camera.processKeyboard(DOWN, dt);
    }
}

void keyboardReleased(unsigned char key, int x, int y)
{
    if (key != ESCAPE) keys[key] = false;
}

void mousePressed(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        pickingEnabled = true;
        pressX = x;
        pressY = y;
    }
}

void mouse(int x, int y)
{
    if (firstMouse) {
        lastX = x;
        lastY = y;
        firstMouse = false;
    }
    
    float dx = x - lastX;
    float dy = lastY - y;
    
    lastX = x;
    lastY = y;
    
    camera.processMouse(dx, dy);
}

int main(int argc, char** argv)
{
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " OBJ_PATH SHADER_PATH" << endl;
        return 0;
    }
    
    path = argv[1];
    shaderPath = argv[2];
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_3_2_CORE_PROFILE | GLUT_MULTISAMPLE);
    glutInitWindowSize(gridX, gridY);
    glutCreateWindow("Project Name");
    
    init();
    
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboardPressed);
    glutKeyboardUpFunc(keyboardReleased);
    glutMouseFunc(mousePressed);
    glutMotionFunc(mouse);
    glutMainLoop();
    
    return 0;
}
