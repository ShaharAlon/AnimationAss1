#include "BasicScene.h"



using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);
    
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

 
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program)}; // empty material

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/bunny.off")};
    sphere1 = Model::Create("sphere", sphereMesh, material);
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;
    };
    autosphere = AutoMorphingModel::Create(*sphere1, morphFunc);
    autosphere->Scale(10);
    autosphere->showWireframe = true;
    autosphere->Translate({-3,0,0});
    camera->Translate(20, Axis::Z);
    root->AddChild(autosphere);
    auto mesh = autosphere->GetMeshList();
    F = mesh[0]->data[0].faces;
    V = mesh[0]->data[0].vertices;
    igl::edge_flaps(F, E, EMAP, EF, EI);
    Q = {};
    VQ = {};    C.resize(E.rows(), V.cols());
    EQ = Eigen::VectorXi::Zero(E.rows());
    Eigen::VectorXd costs(E.rows());
    for (int e = 0; e < E.rows(); e++)
    {
        double cost = e;
        Eigen::RowVectorXd p(1, 3);
        igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
        C.row(e) = p;
        costs(e) = cost;
    }
    for (int e = 0; e < E.rows(); e++)
    {
        Q.emplace(costs(e), e, 0);
    }
    
    
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    //cube->Rotate(0.01f, Axis::All);
}

void BasicScene::KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            Simplification(0.01*Q.size());
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            camera->RotateInSystem(system, 0.1f, Axis::X);
            break;
        case GLFW_KEY_DOWN:
            camera->RotateInSystem(system, -0.1f, Axis::X);
            break;
        case GLFW_KEY_LEFT:
            camera->RotateInSystem(system, 0.1f, Axis::Y);
            break;
        case GLFW_KEY_RIGHT:
            camera->RotateInSystem(system, -0.1f, Axis::Y);
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.05f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.05f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.05f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.05f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.05f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.05f });
            break;
        case GLFW_KEY_H:
            if(autosphere->meshIndex<=1)
                autosphere->meshIndex = 0;
            else
                autosphere->meshIndex -= 1;
             break;
        case GLFW_KEY_L:
            if (autosphere->meshIndex >= autosphere->GetMeshList()[0]->data.size()-1)
                autosphere->meshIndex = autosphere->GetMeshList()[0]->data.size() - 1;
            else
                autosphere->meshIndex += 1;
            break;
        }
    }
}

void BasicScene::Simplification(int numOfEdges) 
{
    bool anyCollapsed = false;
    auto mesh = autosphere->GetMeshList();
    for (int i = 0; i < numOfEdges; i++) {
        if (!igl::collapse_edge(igl::shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
        {
            break;
        }
        anyCollapsed = true;
    }
    if (anyCollapsed) {
        igl::per_vertex_normals(V, F, N);
        T = Eigen::MatrixXd::Zero(V.rows(), 2);
        mesh[0]->data.push_back({ V,F,N,T });
        autosphere->SetMeshList(mesh);
        autosphere->meshIndex = mesh[0]->data.size() - 1;
    }
}

void BasicScene::EdgesCost(const int e, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXi& E, const Eigen::VectorXi&, const Eigen::MatrixXi&, const Eigen::MatrixXi&, double& cost, Eigen::RowVectorXd& p)
{
}
