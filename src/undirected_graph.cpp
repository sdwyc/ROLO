#include "rolo/undirected_graph.h"

UndirectedGraph::UndirectedGraph():
    vertex_num(0), edge_num(0), max_find_time(100){}

UndirectedGraph::UndirectedGraph(const Eigen::Matrix<double, 3, Eigen::Dynamic> &cloud_mat,
                const Eigen::Matrix<int, 2, Eigen::Dynamic> &ti_map,
                const Eigen::Matrix<bool, 1, Eigen::Dynamic> &inlier_mask):
vertex_num(0), edge_num(0), max_find_time(100)
{
    // 添加节点
    populateVertices(cloud_mat);
    // 添加边
    for (size_t i = 0; i < inlier_mask.cols(); ++i) {
      if (inlier_mask(0, i)) {
        addEdge(ti_map(0, i), ti_map(1, i));
      }
    }
    printf("|N| = %d \n", vertex_num);
    printf("|E| = %d \n", edge_num);

}

void UndirectedGraph::addVertex(const int& vertex_id){
    if (vertex_id < vertex_list.size()) {
      printf("Vertex already exists.");
      return;
    } 
    else {
      edge_list.resize(vertex_id + 1);
      vertex_list.resize(vertex_id + 1);
      vertex_num++;
    }
  }

void UndirectedGraph::addEdge(const int& vertex_1, const int& vertex_2){
    if (hasEdge(vertex_1, vertex_2)) {
      printf("Edge exists.");
      return;
    }
    edge_list[vertex_1].push_back(vertex_2);
    edge_list[vertex_2].push_back(vertex_1);
    edge_num++;
}

void UndirectedGraph::populateVertices(const int& vertex_numertices){
    if (vertex_numertices < vertex_num) {
      printf("Vertice are too many.");
      return;
    }
    else{
        edge_list.resize(vertex_numertices);
        vertex_list.resize(vertex_numertices);
        vertex_num = vertex_numertices;
    }
}

void UndirectedGraph::populateVertices(const Eigen::Matrix3Xd &matIn){
    auto cloudSize = matIn.cols();
    if (cloudSize < vertex_num) {
      printf("Vertice are too many.");
      return;
    }
    else{
        edge_list.resize(cloudSize);
        vertex_list.resize(cloudSize);
        vertex_num = cloudSize;
    }
}

void UndirectedGraph::removeEdge(const int& vertex_1, const int& vertex_2){
    if (vertex_1 >= edge_list.size() || vertex_2 >= edge_list.size()) {
      printf("Trying to remove non-existent edge.");
      return;
    }

    edge_list[vertex_1].erase(
        std::remove(edge_list[vertex_1].begin(), edge_list[vertex_1].end(), vertex_2),
        edge_list[vertex_1].end());
    edge_list[vertex_2].erase(
        std::remove(edge_list[vertex_2].begin(), edge_list[vertex_2].end(), vertex_1),
        edge_list[vertex_2].end());

    edge_num--;
}

bool UndirectedGraph::hasEdge(const int& vertex_1, const int& vertex_2){
    if (vertex_1 >= edge_list.size() || vertex_2 >= edge_list.size()) {
      return false;
    }
    auto& connected_vs = edge_list[vertex_1];
    bool exists =
        std::find(connected_vs.begin(), connected_vs.end(), vertex_2) != connected_vs.end();
    return exists;
}

bool UndirectedGraph::hasVertex(const int& vertex_id){
    return vertex_id < vertex_list.size();
}

Eigen::MatrixXi UndirectedGraph::getAdjMatrix(){

    Eigen::MatrixXi adj_matrix(vertex_num, vertex_num);
    for (size_t i = 0; i < vertex_num; ++i) {
      const auto& c_edges = getEdges(i);
      for (size_t j = 0; j < vertex_num; ++j) {
        if (std::find(c_edges.begin(), c_edges.end(), j) != c_edges.end()) {
          adj_matrix(i, j) = 1;
        } else {
          adj_matrix(i, j) = 0;
        }
      }
    }
    return adj_matrix;
}

std::vector<int> UndirectedGraph::findMaxClique(){
    // 格式转换
    std::vector<int> edges;
    std::vector<long long> vertices;
    vertices.push_back(edges.size());
    
    for(auto& it : getVertices()){
      std::vector<int> V_edges = getEdges(it);
      edges.insert(edges.end(), V_edges.begin(), V_edges.end());
      vertices.push_back(edges.size());
    }
    
    pmc::pmc_graph G(vertices, edges);

    // 配置输入
    pmc::input in;
    in.algorithm = 0;
    in.threads = 12;
    in.experiment = 0;
    in.lb = 0;
    in.ub = 0;
    in.param_ub = 0;
    in.adj_limit = 20000;
    in.time_limit = 3000;
    in.remove_time = 4;
    in.graph_stats = false;
    in.verbose = false;
    in.help = false;
    in.MCE = false;
    in.decreasing_order = false;
    in.heu_strat = "kcore";
    in.vertex_search_order = "deg";

    // vector to represent max clique
    vector<int> C;
    // upper-bound of max clique
    G.compute_cores();
    auto max_core = G.get_max_core();


    if (in.ub == 0) {
      in.ub = max_core + 1;
    }

    // lower-bound of max clique
    if (in.lb == 0 && in.heu_strat != "0") { // skip if given as input
      pmc::pmc_heu maxclique(G, in);
      in.lb = maxclique.search(G, C);
    }

    assert(in.lb != 0);
    if (in.lb == 0) {
      // This means that max clique has a size of one
      printf("Max clique lower bound equals to zero. Abort.");
      return C;
    }

    if (in.lb == in.ub) {
      return C;
    }
    // 寻找最大团
    if (G.num_vertices() < in.adj_limit) {
      G.create_adj();
      pmc::pmcx_maxclique finder(G, in);
      finder.search_dense(G, C);
    } else {
      pmc::pmcx_maxclique finder(G, in);
      finder.search(G, C);
    }

    return C;
}