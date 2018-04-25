// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
#define NUM_OF_ITERATIONS 10

#include <rosconsole/macros_generated.h>
#include <ros/assert.h>
#include <ros/ros.h>
#include "../include/object_detect/GraphOptimizer_G2O.h"

GraphOptimizer_G2O::GraphOptimizer_G2O()
{
    // Choice for Method here is GaussNewton or LevenbergMarquardt
    // LevenbergMarquardt is our non-linear optimization method
    //optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
    // verbose info during optimization 
    optimizer.setVerbose(false);

    // LinearSolverCholmod is basic solver for Ax=b
    // LinearSolverCholmod is expecting a <MatrixType> and will be used to internally
    // solve the linear subproblems by using Cholesky decomposition
    // Assumption: Sparse, not Dense
    linearSolver = std::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    solver_ptr = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr)));

    // Set vertex and edge index to 0
    vertexIndex = 0;
    nextEdgeIndex = 0;
}

int GraphOptimizer_G2O::addVertex(const Eigen::Vector3d &vertexPose)
{
    // Transform Eigen::Vector3f into translation and rotation for g2o
    //Eigen::Vector2d t(vertexPose(0),vertexPose(1));
    //double r = vertexPose(2);

    // Vertex pose is set to r and t
    //g2o::SE2 pose(r,t);
    ROS_INFO_STREAM("Adding pose...");
    g2o::SE2 pose(vertexPose(0),vertexPose(1),vertexPose(2));
    // Setup node with g2o::VertexSE2 i x y theta
    g2o::VertexSE2 *vc = new g2o::VertexSE2();
    vc->setEstimate(pose);
    //vc->estimate() = pose;

    // Vertex ID
    vc->setId(vertexIndex);

    // Set first pose fixed
    if (vertexIndex==0)
    {
        vc->setFixed(true);
    }

    // Add to optimizer
    optimizer.addVertex(vc);
    vertices.push_back(vc);
    ROS_INFO_STREAM("Done.");
    // Update vertex index
    ++vertexIndex;

    // Return 
    return vertexIndex-1;
}

void GraphOptimizer_G2O::addEdge(const int fromIndex, const int toIndex, const Eigen::Vector3d &relativePose, const Eigen::Matrix<double,3,3> &infoMatrix)
{
    ROS_INFO_STREAM("Adding edge...");
    // Transform Eigen::Vector3f into translation and rotation for g2o
    //g2o::Vector2d t(relativePose(0),relativePose(1));
    //double r = relativePose(2);

    // Relative transformation
    g2o::SE2 relativeTransform(relativePose(0),relativePose(1),relativePose(2));
    // Setup Edge
    g2o::EdgeSE2 *edge = new g2o::EdgeSE2;
    edge->vertices()[0] = optimizer.vertex(fromIndex);
    edge->vertices()[1] = optimizer.vertex(toIndex);

    // Edge ID
    edge->setId(nextEdgeIndex);
    ++nextEdgeIndex;

    ROS_INFO_STREAM("Done adding edge.");
    // setMeasurement takes a SE2 object relativeTransform
    //edge->setMeasurementFromState();
    edge->setMeasurement(relativeTransform);
    //edge->setMeasurement(relativeTransform);

    // Set the information matrix to identity
    edge->setInformation(infoMatrix);

    // Added these in based on point-point code in g2o demo
    // Extra params to play around with
    //edge->setRobustKernel(true);
    //edge->setHuberWidth(0.01);

    // Add edge to the optimizer
    edges.push_back(edge);
    optimizer.addEdge(edge);
}
void GraphOptimizer_G2O::addOdometryEdge(const int fromIndex, const int toIndex, const Eigen::Matrix<double,3,3> &infoMatrix){
    // Setup Edge
    g2o::EdgeSE2 *edge = new g2o::EdgeSE2;
    edge->vertices()[0] = optimizer.vertex(fromIndex);
    edge->vertices()[1] = optimizer.vertex(toIndex);

    // Edge ID
    edge->setId(nextEdgeIndex);
    ++nextEdgeIndex;

    ROS_INFO_STREAM("Done adding edge.");
    // setMeasurement takes a SE2 object relativeTransform
    edge->setMeasurementFromState();
    //edge->setMeasurement(relativeTransform);

    // Set the information matrix to identity
    edge->setInformation(infoMatrix);

    // Added these in based on point-point code in g2o demo
    // Extra params to play around with
    //edge->setRobustKernel(true);
    //edge->setHuberWidth(0.01);
    //edge->
    // Add edge to the optimizer
    edges.push_back(edge);
    optimizer.addEdge(edge);
}
void GraphOptimizer_G2O::optimizeGraph()
{

    // Check if graph is empty
    if (optimizer.edges().size() == 0)
    {
        std::cout << "Graph is empty!\n";
    }
    
    // Prepare and run optimization on graph
    optimizer.initializeOptimization();
 
    // Sets up initial damping factor for Levenberg-Marquardt
    //optimizer.setUserLambdaInit(0.01);
    // We also have the option to compute it
    // optimizer.computeLambdaInit();

    // Computes the error vectors of all edges in the activeSet,
    // and then caches them. With robustKernel on, this robustifyError()
    optimizer.computeActiveErrors();
    std::cout << "Initial chi^2 = " << FIXED(optimizer.chi2()) << std::endl;

    // Can turn on if necessary
    optimizer.setVerbose(false);
    optimizer.optimize(NUM_OF_ITERATIONS);
}

void GraphOptimizer_G2O::saveGraph(std::string fileName)
{
    optimizer.save(fileName.c_str(),0);
}
