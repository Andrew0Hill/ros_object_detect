// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//

#ifndef GRAPHOPTIMIZER_G2O
#define GRAPHOPTIMIZER_G2O

#include <vector>
#include <Eigen/Core>
#include <string>
//#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
//#include "g2o/solvers/cholmod/linear_solver_cholmod.h"  // Cholesky Decomposition
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include <memory>

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1,-1>> SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

class GraphOptimizer_G2O
{
private:

    /*
    We first need to allocate the optimizer. We create a SparseOptimizer optimizer
    derived from OptimizableGraph which is an abstract class that 
    represents one optimization problem. It specializes the general graph 
    to contain special vertices and edges. The vertices represent parameters 
    that can be optimized, while the edges represent constraints generated from 
    sensor measurements. We assume the graph to be sparse.
    */
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<SlamLinearSolver>  linearSolver;
    //g2o::BlockSolverX::LinearSolverType *linearSolver_ptr;
    std::unique_ptr<SlamBlockSolver> blockSolver;
    std::unique_ptr<SlamBlockSolver> solver_ptr;
    SlamLinearSolver sls;
public:
    GraphOptimizer_G2O();
    std::vector<g2o::VertexSE2*> vertices;
    std::vector<g2o::EdgeSE2*> edges;
    int vertexIndex;
    int nextEdgeIndex;
    // Add a new vertex to the graph.
    // INPUT: Column vector of type float giving the pose of the robot as an 
    // OUTPUT: Returns an int index of the new vertex
    int addVertex(const Eigen::Vector3d &pose);

    // Add an edge that defines a spatial constraint between fromIndex and toIndex
    // With the information matrix, computes a weight associated with the added edge.
    // INPUT: Index of a fromVertex and toVertex, Column vector giving relative pose.
    // 3x3 matrix of type float Information Matrix.
    void addEdge(const int fromIndex, const int toIndex, const Eigen::Vector3d &relativePose, const Eigen::Matrix<double,3,3> &infoMatrix);

    // Calls a graph optimization process from g2o::SparseOptimizer
    // to determine the pose configuration that best satisfies the edge constraints
    void optimizeGraph();

    // Saves the graph to file.
    void saveGraph(std::string fileName);
};

#endif
