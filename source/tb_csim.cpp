#include "subgraphIsomorphism.hpp"
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cassert>
#include <unistd.h>
#include <unordered_map>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring>
#include <tuple>
#include <string>
#include <cstdint>
#include <chrono>
#include <cmath>

#include <ap_int.h>
#include <hls_stream.h>
#include "Parameters.hpp"
#include "debug.hpp"

#if SOFTWARE_PREPROC
#include "preprocess.hpp"
#endif /* SOFTWARE_PREPROC */

template <size_t NODE_W,
         size_t LAB_W,
         size_t BURST_SIZE,
         size_t RESULT_SPACE,
         size_t MAX_QDATA>
void load_datagraphs(
        row_t *edge_buf,
        std::string datafile,
        unsigned long &dynfifo_space,
        unsigned long &numDataEdges) {
    unsigned long numDataVertices;
    unsigned long edge_buf_p = 0;
    edge_t edge;
    
    /* Remove "../" to make paths correct */
    datafile = datafile.substr(3);
    
    std::ifstream fData(datafile);
    
    if (!fData.is_open()) {
        std::cout << "Datagraph file opening failed.\n";
        return;
    }

    std::string fLine{};
    std::unordered_map<unsigned long, unsigned long> vToLabelData;

    std::getline(fData, fLine);
    sscanf(fLine.c_str(), "%*c %lu %lu", &numDataVertices, &numDataEdges);

    // Find space for the graph and align it to BURST_SIZE
    dynfifo_space = numDataEdges + MAX_QDATA;
    dynfifo_space = dynfifo_space - (dynfifo_space % BURST_SIZE) + BURST_SIZE;
    if (dynfifo_space > RESULT_SPACE) {
        std::cout << "Not enough space for dynamic fifo.\n";
        return;
    }
    dynfifo_space = RESULT_SPACE - dynfifo_space;
    edge_buf_p = dynfifo_space;

    /* Store data labels */
    for(int count = 0; count < numDataVertices; count++) {    
        unsigned long node_t, label_t;
        std::getline(fData, fLine);
        sscanf(fLine.c_str(), "%*c %lu %lu %*u", &node_t, &label_t);
        vToLabelData.insert(std::make_pair(node_t, label_t));
    }
    
    std::cout << "Loading datagraph in DDR..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    /* Stream edges */
    for(int count = 0; count < numDataEdges; count++) {    
        unsigned long nodesrc_t, nodedst_t;
        std::getline(fData, fLine);
        sscanf(fLine.c_str(), "%*c %lu %lu", &nodesrc_t, &nodedst_t);
        edge.labelsrc = vToLabelData.at(nodesrc_t); 
        edge.labeldst = vToLabelData.at(nodedst_t);
        edge.src = nodesrc_t;
        edge.dst = nodedst_t;
        memcpy(&edge_buf[edge_buf_p++], &edge, sizeof(row_t));
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Done in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms." << std::endl;

    fData.close();
}

template <size_t NODE_W,
         size_t LAB_W,
         size_t MAX_QDATA>
std::pair<int, int> load_querygraphs(
        row_t *edge_buf,
        std::string queryfile,
        const unsigned long dynfifo_space,
        unsigned short &numQueryVertices,
        unsigned short &numQueryEdges,
        unsigned short &tableListLength,
        unsigned long numDataEdges) {
    unsigned long numDataVertices;
    unsigned long edge_buf_p = numDataEdges + dynfifo_space;
    std::string fLine{};
    std::unordered_map<unsigned long, unsigned long> vToLabelQuery;
    std::vector<std::vector<unsigned long>> adjacency_list;
    std::vector<std::tuple<int, int, int, int>> edge_list; // Stores edges in format (src, dst, labelsrc, labeldst)
    std::vector<std::tuple<int, int, bool>> tablelist; // To store unique edges for memory overflow check
    edge_t edge;
    
    /* Remove "../" to make paths correct */
    queryfile = queryfile.substr(3);
    
    /* Query file */
    std::ifstream fQuery(queryfile);

    // Get the current path
    char currentPath[FILENAME_MAX];
    
    std::cout << "Querygraph: " << queryfile << std::endl; 
    if (!fQuery.is_open()){
        std::cout << "Query file opening failed.\n";
        return {-1, 0};
    }

    /* Read vertices and edges cardinality */
    std::getline(fQuery, fLine);
    sscanf(fLine.c_str(), "%*c %hu %hu", &numQueryVertices, &numQueryEdges);
    std::cout << "Query vertices: " << numQueryVertices << ", Query edges: " << numQueryEdges << std::endl;

    adjacency_list.resize(numQueryVertices);    // resize adjacency list after reading correct size
    assert(MAX_QDATA >= numQueryEdges + numQueryVertices);
    
    /* Store query labels */
    for (int count = 0; count < numQueryVertices; count++) {
        unsigned long node_t, label_t;
        std::getline(fQuery, fLine);
        sscanf(fLine.c_str(), "%*c %lu %lu %*u", &node_t, &label_t);
        vToLabelQuery.insert(std::make_pair(node_t, label_t));
    }

    /* Stream edges */
    for(int count = 0; count < numQueryEdges; count++) {    
        unsigned long nodesrc_t, nodedst_t;
        std::getline(fQuery, fLine);
        sscanf(fLine.c_str(), "%*c %lu %lu", &nodesrc_t, &nodedst_t);
        if (nodesrc_t >= adjacency_list.size()) {
          adjacency_list.resize(nodesrc_t + 1);
        }
        if (nodedst_t >= adjacency_list.size()) {
          adjacency_list.resize(nodedst_t + 1);
        }
        adjacency_list[nodesrc_t].push_back(nodedst_t);
        adjacency_list[nodedst_t].push_back(nodesrc_t);
        int labelsrc = vToLabelQuery.at(nodesrc_t);
        int labeldst = vToLabelQuery.at(nodedst_t);

        edge_list.emplace_back(nodesrc_t, nodedst_t, labelsrc, labeldst);

        // Counting number of tables for memory overflow check
        bool direction = (nodesrc_t < nodedst_t);
        std::tuple<int, int, bool> tupleedge = std::make_tuple(labelsrc, labeldst, direction);

        if (find(tablelist.begin(), tablelist.end(), tupleedge) == tablelist.end()) {
            tablelist.push_back(tupleedge);
        }
    }
    fQuery.close();

    // Ordering query nodes based on degrees. The starting node is the one with the highest degree.
    // Then, the node with the highest number of neighbors in the already ordered set is selected.
    // Selecting the node with the highest degree as the starting node
    int max_degree = 0;
    int start_node = 0;
    std::vector<int> order;
    std::vector<int> query_vertices(numQueryVertices);

    for (int v = 0; v < numQueryVertices; ++v) {
        query_vertices[v] = v; // Initializing the query vertices list
        int degree = adjacency_list[v].size();
        if (degree > max_degree) {
            max_degree = degree;
            start_node = v;
        }
    }

    order.push_back(start_node);
    query_vertices.erase(remove(query_vertices.begin(), query_vertices.end(), start_node), query_vertices.end());

    for (int x = 0; x < numQueryVertices - 1; ++x) {
        int max_neigh = 0;
        int following = query_vertices[0];
        for (int candidate : query_vertices) {
            int neighbors_already_matched = 0;
            for (int neighbor : adjacency_list[candidate]) {
                if (find(order.begin(), order.end(), neighbor) != order.end()) {
                    neighbors_already_matched++;
                }
            }

            if (neighbors_already_matched > max_neigh) {
                max_neigh = neighbors_already_matched;
                following = candidate;
            }
            // If two nodes have the same number of neighbors already matched,
            // select the one with the highest degree
            else if (neighbors_already_matched == max_neigh) {
                if (adjacency_list[candidate].size() > adjacency_list[following].size()) {
                    following = candidate;
                }
            }
        }

        if (max_neigh == 0) {
            std::cout << "Error: query graph is not connected." << std::endl;
            return {-1, 0};
        }

        query_vertices.erase(remove(query_vertices.begin(), query_vertices.end(), following), query_vertices.end());
        order.push_back(following);
    }

    /* Stream matching order */
    std::cout << "Query vertex order: [";
    for(int count = 0; count < numQueryVertices; count++) {
        // Initialize all fields for a vertex instruction before writing vertex order to buffer 
        edge.src = order[count];
        edge.dst = 0;
        edge.labelsrc = 0;
        edge.labeldst = 0;

        std::cout << order[count] << " ";
        memcpy(&edge_buf[edge_buf_p++], &edge, sizeof(row_t));
    }
    std::cout << "]" << std::endl;
    
    /* Stream edges */
    // Fill all fields from edge_list
    for(int count = 0; count < numQueryEdges; count++) {    
        unsigned long nodesrc_t, nodedst_t;
        auto tuple_edge = edge_list[count];
        edge.src = std::get<0>(tuple_edge);
        edge.dst = std::get<1>(tuple_edge);
        edge.labelsrc = std::get<2>(tuple_edge);
        edge.labeldst = std::get<3>(tuple_edge);
        memcpy(&edge_buf[edge_buf_p++], &edge, sizeof(row_t));
    }

    std::cout << "\n--- KERNEL DATA VERIFICATION ---" << std::endl;

    // Verify the final calculated order
    std::cout << "Final Vertex Order Sent to Kernel: [ ";
    for(int node : order) {
        std::cout << node << " ";
    }
    std::cout << "]" << std::endl;

    // Dump the raw buffer contents that the kernel will read
    std::cout << "Raw Buffer Content (first 20 entries):" << std::endl;
    std::cout << "Type      \t|\t src \t|\t dst \t|\t lsrc \t|\t ldst" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;

    // Calculate where the query data starts in the main buffer
    unsigned long query_start_p = numDataEdges + dynfifo_space;

    // Loop through and print the first few instructions
    for (int i = 0; i < (numQueryVertices + numQueryEdges) && i < 20; ++i) {
        // Cast raw row_t buffer data back to an edge_t to inspect it
        edge_t temp_edge;
        memcpy(&temp_edge, &edge_buf[query_start_p + i], sizeof(edge_t));

        if (i < numQueryVertices) {
            std::cout << "Vertex Inst\t|\t "
                    << temp_edge.src << "\t|\t "
                    << temp_edge.dst << "\t|\t "
                    << temp_edge.labelsrc << "\t|\t "
                    << temp_edge.labeldst << std::endl;
        } else {
            std::cout << "Edge Inst  \t|\t "
                    << temp_edge.src << "\t|\t "
                    << temp_edge.dst << "\t|\t "
                    << temp_edge.labelsrc << "\t|\t "
                    << temp_edge.labeldst << std::endl;
        }
    }
    std::cout << "--- END KERNEL DATA VERIFICATION ---\n" << std::endl;

    tableListLength = tablelist.size();
    return {0, max_degree};
}

int main(int argc, char** argv) {
    std::cout << "--- C-Simulation Testbench ---" << std::endl;

    /* Test Case Definition */
    std::string datagraph = "../dataset_example/label_5/data/graph_simple3.RM.csv";
    std::string querygraph = "../dataset_example/label_5/queries2/query_simple2.RM.csv";
    long unsigned int golden_result = 2;

    /* Parameters */
    unsigned short nQV = 0;
    unsigned short nQE = 0;
    unsigned short tablelist_length = 0;
    unsigned long nDE = 0;
    unsigned long dynfifo_space = 0;

    /* Allocate Memory */
    row_t* htb_buf = new row_t[HASHTABLES_SPACE];
    row_t* res_buf = new row_t[RESULTS_SPACE];
    row_t* bloom_p = new row_t[BLOOM_SPACE];

    /* Zero-initialize buffers */ 
    memset(htb_buf, 0, HASHTABLES_SPACE * sizeof(row_t));
    memset(res_buf, 0, RESULTS_SPACE * sizeof(row_t));
    memset(bloom_p, 0, BLOOM_SPACE * sizeof(row_t));

    /* Load Graph Data into Buffers */
    load_datagraphs<VERTEX_WIDTH_BIT, LABEL_WIDTH, DYN_FIFO_BURST, RESULTS_SPACE, MAX_QUERYDATA>(
        res_buf, datagraph, dynfifo_space, nDE);

    auto query_info = load_querygraphs<VERTEX_WIDTH_BIT, LABEL_WIDTH, MAX_QUERYDATA>(
        res_buf, querygraph, dynfifo_space, nQV, nQE, tablelist_length, nDE);

    if (query_info.first < 0) {
        std::cout << "Error loading query graph." << std::endl;
        return 1;
    }

    int max_degree = query_info.second;

    /* Dynamic H1 and H2 calculation */
    std::cout << "INFO: Dynamically calculating h1 and h2..." << std::endl;
    unsigned char h1 = static_cast<unsigned char>(0.4 * log(5e7 * nDE)) + 2;
    unsigned char h2 = static_cast<unsigned char>(std::min(max_degree + 1, 7));
    if (h1 + h2 <= 14) {
        h2 = 14 - h1;
        std::cout << "INFO: Adjusted H2 to " << (int)h2 << " to meet minimum hash width sum requirement." << std::endl;
    }
    std::cout << "INFO: Calculated H1=" << (int)h1 << ", H2=" << (int)h2 << std::endl;

    /* Call Kernel Function */
    long unsigned int result_actual = 0;
    unsigned int dynfifo_overflow = 0;
    
    /* Dummy variables for debug interface (unused) */
    volatile unsigned int debug_endpreprocess_s = 0;
    unsigned long p_hits[5] = {0};
    unsigned long p_reqs[7] = {0};

    std::cout << "Starting C Simulation..." << std::endl;

    subgraphIsomorphism(
        htb_buf, htb_buf, htb_buf, htb_buf,
        bloom_p,
        res_buf,
        nQV, nQE, nDE,
        h1, h2,
        dynfifo_space,
        dynfifo_overflow,
#if DEBUG_INTERFACE
        debug_endpreprocess_s,
        p_hits[0], p_hits[1], p_hits[2], p_hits[3], p_hits[4],
        p_reqs[0], p_reqs[1], p_reqs[2], p_reqs[3], p_reqs[4], p_reqs[5], p_reqs[6],
#endif
        result_actual
    );

    std::cout << "C Simulation Finished." << std::endl;

    /* Check Results */
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "Golden Result: " << golden_result << std::endl;
    std::cout << "Actual Result: " << result_actual << std::endl;

    bool match = (result_actual == golden_result);
    std::cout << "Test " << (match ? "PASSED" : "FAILED") << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    /* Clean up memory */
    delete[] htb_buf;
    delete[] res_buf;
    delete[] bloom_p;

    return (match ? 0 : 1);
}