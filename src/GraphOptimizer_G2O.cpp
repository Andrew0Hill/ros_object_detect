// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
#define NUM_OF_ITERATIONS 10

#include "../src/GraphOptimizer_G2O.h"

GraphOptimizer_G20::GraphOptimizer_G20()
{
    // Choice for Method here is GaussNewton or LevenbergMarquardt
    // LevenbergMarquardt is our non-linear optimization method
    optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);

    // verbose info during optimization 
    optimizer.setVerbose(false);

    // LinearSolverCholmod is basic solver for Ax=b
    // LinearSolverCholmod is expecting a <MatrixType> and will be used to internally
    // solve the linear subproblems by using Cholesky decomposition
    // Assumption: Sparse, not Dense
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
    solver_ptr = new g2o::BlockSolverX(&optimizer, linearSolver);

    optimizer.setAlgorithm(solver_ptr);

    // Set vertex and edge index to 0
    vertexIndex = 0;
    nextEdgeIndex = 0;
}

int GraphOptimizer_G2O::addVertex(Eigen::Vector3f &vertexPose)
{
    // Transform Eigen::Vector3f into translation and rotation for g2o
    g2o::Vector2d t(vertexPose(0),vertexPose(1));
    double r = vertexPose(2);

    // Vertex pose is set to r and t
    g2o::SE2 pose(r,t);

    // Setup node with g2o::VertexSE2 i x y theta
    g2o::VertexSE2 *vc = new g2o::VertexSE2();
    vc->estimate() = pose;

    // Vertex ID
    vc->setId(vertexIndex);

    // Set first pose fixed
    if (vertexIndex==0)
    {
        vc->setFixed(true);
    }

    // Add to optimizer
    optimizer.addVertex(vc);

    // Update vertex index
    ++vertexIndex;

    // Return 
    return vertexIndex-1;
}

void GraphOptimizer_G2O::addEdge(const int fromIndex, const int toIndex, Eigen::Vector3f &relativePose, Eigen::Matrix<double,3,3> &infoMatrix)
{
    // Transform Eigen::Vector3f into translation and rotation for g2o
    g2o::Vector2d t(relativePose(0),relativePose(1));
    double r = relativePose(2); 

    // Relative transformation
    g2o::SE2 relativeTransform(r,t);

    // Setup Edge
    g2o::EdgeSE2 *edge = new g2o::EdgeSE2;
    edge->vertices()[0] = optimizer.vertex(fromIndex);
    edge->vertices()[1] = optimizer.vertex(toIndex);

    // Edge ID
    edge->setId(nextEdgeIndex);
    ++nextEdgeIndex;

    // setMeasurement takes a SE2 object relativeTransform
    edge->setMeasurement(relativeTransform);

    // Set the information matrix to identity
    edge->setInformation(infoMatrix);

    // Added these in based on point-point code in g2o demo
    // Extra params to play around with
    edge->setRobustKernel(true);
    edge->setHuberWidth(0.01);

    // Add edge to the optimizer
    optimizer.addEdge(edge);
}

void optimizeGraph()
{
    // Check if graph is empty
    if (optimizer.edges().size() == 0)
    {
        cout << "Graph is empty!\n";
    }
    
    // Prepare and run optimization on graph
    optimizer.initializeOptimization();
 
    // Sets up initial damping factor for Levenberg-Marquardt
    optimizer.setUserLambdaInit(0.01);
    // We also have the option to compute it
    // optimizer.computeLambdaInit();

    // Computes the error vectors of all edges in the activeSet,
    // and then caches them. With robustKernel on, this robustifyError()
    optimizer.computeActiveErrors();
    cout << "Initial chi^2 = " << FIXED(optimizer.chi2()) << endl;

    // Can turn on if necessary
    optimizer.setVerbose(false);
    optimizer.optimize(NUM_OF_ITERATIONS);
}

void saveGraph(std::string fileName)
{
    optimizer.save(fileName.c_str(),0)
}
