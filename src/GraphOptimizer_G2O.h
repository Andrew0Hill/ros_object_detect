// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//

#ifndef GRAPHOPTIMIZER_G2O
#define GRAPHOPTIMIZER_G2O

#include <vector>
#include <Eigen/Core>
#include <string>
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"  // Cholesky Decomposition
#include "g2o/types/slam2d/g2o_types_slam2d.h"

class GraphOptimizer_G2O
{
private:
    int vertexIndex;
    /*
    We first need to allocate the optimizer. We create a SparseOptimizer optimizer
    derived from OptimizableGraph which is an abstract class that 
    represents one optimization problem. It specializes the general graph 
    to contain special vertices and edges. The vertices represent parameters 
    that can be optimized, while the edges represent constraints.
    */
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver_ptr;
    g2o::BlockSolverX *solver_ptr;

public:
    GraphOptimizer_G20();
    
    // Add a new vertex to the graph.
    // INPUT: Column vector of type float giving the pose of the robot as an 
    // OUTPUT: Returns an int index of the new vertex
    int addVertex(Eigen::Vector3f &pose);

    // Add an edge that defines a spatial constraint between fromIndex and toIndex
    // With the information matrix, computes a weight associated with the added edge.
    // INPUT: 6x6 matrix of type float Information Matrix
    void addEdge(const int fromIndex, const int toIndex, Eigen::Vector3f &relativePose, Eigen::Matrix<double,6,6> &infoMatrix);

    // Calls a graph optimization process from g2o::SparseOptimizer
    // to determine the pose configuration that best satisfies the edge constraints
    void optimizeGraph();

    // Saves the graph to file.
    void saveGraph(std::string fileName);
};

#endif