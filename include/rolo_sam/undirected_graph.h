#ifndef UNDIRECTED_GRAPH_
#define UNDIRECTED_GRAPH_

#include <unordered_set>
#include <map>
#include <vector>
#include <Eigen/Core>
#include <pmc/pmc.h>

class UndirectedGraph{
private:
    int vertex_num;
    int edge_num;
    std::vector<std::vector<int>> edge_list;
    std::vector<int> vertex_list;
    double max_find_time;
    
public:
    UndirectedGraph();

    UndirectedGraph(const Eigen::Matrix<double, 3, Eigen::Dynamic> &cloud_mat,
                    const Eigen::Matrix<int, 2, Eigen::Dynamic> &ti_map,
                    const Eigen::Matrix<bool, 1, Eigen::Dynamic> &inlier_mask);

    ~UndirectedGraph(){}

    void addVertex(const int& vertex_id);

    void addEdge(const int& vertex_1, const int& vertex_2);

    void populateVertices(const int& num_vertices);

    void populateVertices(const Eigen::Matrix3Xd &matIn);

    void removeEdge(const int& vertex_1, const int& vertex_2);

    bool hasEdge(const int& vertex_1, const int& vertex_2);

    bool hasVertex(const int& vertex_id);

    int getEdgeNum(){ return edge_num; }

    int getVertexNum(){ return vertex_num; }

    std::vector<int>& getEdges(int id){ return edge_list[id]; }

    std::vector<int> getVertices(){ return vertex_list; }

    std::vector<std::vector<int>> getAdjList(){ return edge_list; }
    
    Eigen::MatrixXi getAdjMatrix();
    
    // Max Clique Problem
    std::vector<int> findMaxClique();
    
};

#endif // UNDIRECTED_GRAPH_