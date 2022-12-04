#pragma once

#include "Scene.h"
#include "AutoMorphingModel.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include <igl/collapse_edge.h>
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "per_vertex_normals.h"


class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void Simplification(int numOfEdges);
    void EdgesCost(const int e, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F/*F*/, const Eigen::MatrixXi& E, const Eigen::VectorXi& /*EMAP*/, const Eigen::MatrixXi& /*EF*/, const Eigen::MatrixXi& /*EI*/, double& cost, Eigen::RowVectorXd& p);

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> sphere1;
    std::shared_ptr<cg3d::AutoMorphingModel> autosphere;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F, E, EF, EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V,C,N,T;
    igl::min_heap<std::tuple<double, int, int>> Q;
    std::vector<std::tuple<int, double>> VQ;
};
