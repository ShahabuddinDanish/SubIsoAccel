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
#include <stdexcept>
#include <iomanip>
#include <utility>
#include <cmath>

#include <ap_int.h>
#include <hls_stream.h>
#include "Parameters.hpp"
#include "debug.hpp"

#if SOFTWARE_PREPROC
#include "preprocess.hpp"
#endif /* SOFTWARE_PREPROC */

struct TestEntry {
    std::string querygraph;
    std::string golden;
    std::string h1;
    std::string h2;
};

template <size_t NODE_W,
         size_t LAB_W,
         size_t BURST_SIZE,
         size_t RESULT_SPACE,
         size_t MAX_QDATA>
unsigned long load_datagraphs(
        row_t *edge_buf,
        std::string datafile,
        unsigned long &dynfifo_space,
        unsigned long &numDataEdges,
        unsigned short numQueryVertices,
        unsigned short numQueryEdges)
{
    unsigned long numDataVertices;
    edge_t edge;
    
    /* Remove "../" to make paths correct */
    datafile = datafile.substr(3);
    std::ifstream fData(datafile);
    if (!fData.is_open()){
        std::cout << "Datagraph file opening failed.\n";
        return 0;
    }

    std::string fLine{};
    std::unordered_map<unsigned long, unsigned long> vToLabelData;

    std::getline(fData, fLine);
    sscanf(fLine.c_str(), "%*c %lu %lu", &numDataVertices, &numDataEdges);

    // Calculate total number of 128-bit instructions needed to store
    unsigned long total_128bit_items = numDataEdges + numQueryVertices + numQueryEdges;
    // Calculate how many 512-bit words are needed to store instructions
    unsigned long num_512bit_words_for_graph = (total_128bit_items + INSTR_PER_WORD - 1) / INSTR_PER_WORD;

    // The dynamic FIFO gets the remaining space. Its starting address is 0.
    // The graph data will be placed at the end of the buffer.
    dynfifo_space = RESULTS_SPACE - num_512bit_words_for_graph;
    if (dynfifo_space > RESULT_SPACE){
        std::cout << "Not enough space for dynamic fifo.\n";
        return -1;
    }
    unsigned long edge_buf_p = dynfifo_space; // The write pointer starts where the FIFO space ends.

    /* Store data labels */
    for(int count = 0; count < numDataVertices; count++){    
        unsigned long node_t, label_t;
        std::getline(fData, fLine);
        sscanf(fLine.c_str(), "%*c %lu %lu %*u", &node_t, &label_t);
        vToLabelData.insert(std::make_pair(node_t, label_t));
    }
    
    row_t temp_word; 
    int pack_counter = 0;
    memset(&temp_word, 0, sizeof(row_t));       // initially set the buffer to zero

    std::cout << "Loading and packing datagraph in DDR..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    /* Stream edges */
    for(int count = 0; count < numDataEdges; count++){    
        unsigned long nodesrc_t, nodedst_t;
        std::getline(fData, fLine);
        sscanf(fLine.c_str(), "%*c %lu %lu", &nodesrc_t, &nodedst_t);

        edge.labelsrc = vToLabelData.at(nodesrc_t); 
        edge.labeldst = vToLabelData.at(nodedst_t);
        edge.src = nodesrc_t;
        edge.dst = nodedst_t;

        // pack the 128-bit edge into temp_word and cast temp_word to a char* to do byte-level pointer access
        memcpy( ((char*)&temp_word) + (pack_counter * sizeof(edge_t)), 
                &edge, 
                sizeof(edge_t) );
        
        pack_counter++;

        // if temp_word is full, write it to the main buffer and reset
        if (pack_counter == INSTR_PER_WORD) {
            // Copy the full temp_word to the edge buffer
            memcpy(&edge_buf[edge_buf_p++], &temp_word, sizeof(row_t));
            
            // Reset the counter and the temp_word
            pack_counter = 0;
            memset(&temp_word, 0, sizeof(row_t));
        }
    }

    // after the loop, handle any leftover edges that didn't make a full batch of 4
    if (pack_counter > 0) {
        memcpy(&edge_buf[edge_buf_p++], &temp_word, sizeof(row_t));
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Done in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms." << std::endl;

    fData.close();
    return edge_buf_p; // Return the next free index
}

template <size_t NODE_W,
         size_t LAB_W,
         size_t MAX_QDATA>
std::pair<int, int> load_querygraphs(
        row_t *edge_buf,
        unsigned long& edge_buf_p,
        std::string queryfile,
        const unsigned long dynfifo_space,
        unsigned short &numQueryVertices,
        unsigned short &numQueryEdges,
        unsigned long numDataEdges,
        unsigned short &tableListLength)
{
    unsigned long numDataVertices;
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
        std::cout << "Query file opening failed: " << queryfile << std::endl;
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
    for(int count = 0; count < numQueryEdges; count++){    
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

    row_t temp_word;
    int pack_counter = 0;
    memset(&temp_word, 0, sizeof(row_t));

    /* Stream matching order */
    std::cout << "Query vertex order: [";
    for(int count = 0; count < numQueryVertices; count++){
        // Initialize all fields for a vertex instruction before writing vertex order to buffer 
        edge.src = order[count];
        edge.dst = 0;
        edge.labelsrc = 0;
        edge.labeldst = 0;

        std::cout << order[count] << " ";

        memcpy( ((char*)&temp_word) + (pack_counter * sizeof(edge_t)), 
                &edge, 
                sizeof(edge_t) );
        pack_counter++;

        if (pack_counter == INSTR_PER_WORD) {
            memcpy(&edge_buf[edge_buf_p++], &temp_word, sizeof(row_t));
            pack_counter = 0;
            memset(&temp_word, 0, sizeof(row_t));
        }
    }
    std::cout << "]" << std::endl;
    
    /* Stream edges */
    // Fill all fields from edge_list
    for(int count = 0; count < numQueryEdges; count++){    
        auto tuple_edge = edge_list[count];
        edge.src = std::get<0>(tuple_edge);
        edge.dst = std::get<1>(tuple_edge);
        edge.labelsrc = std::get<2>(tuple_edge);
        edge.labeldst = std::get<3>(tuple_edge);

        memcpy( ((char*)&temp_word) + (pack_counter * sizeof(edge_t)), 
                &edge, 
                sizeof(edge_t) );
        pack_counter++;

        if (pack_counter == INSTR_PER_WORD) {
            memcpy(&edge_buf[edge_buf_p++], &temp_word, sizeof(row_t));
            pack_counter = 0;
            memset(&temp_word, 0, sizeof(row_t));
        }
    }
    
    if (pack_counter > 0) {
        memcpy(&edge_buf[edge_buf_p++], &temp_word, sizeof(row_t));
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
    unsigned long num_data_words = (numDataEdges + INSTR_PER_WORD - 1) / INSTR_PER_WORD;
    unsigned long query_start_word_p = dynfifo_space + num_data_words;

    // Loop through and print the first few instructions
    for (int i = 0; i < (numQueryVertices + numQueryEdges) && i < 20; ++i) {

        // Calculate which 512-bit word and which 128-bit slot to read
        unsigned long word_idx = query_start_word_p + (i / INSTR_PER_WORD);
        unsigned long slot_idx = i % INSTR_PER_WORD;

        // Cast raw row_t buffer data back to an edge_t to inspect it
        row_t temp_word = edge_buf[word_idx];
        edge_t temp_edge;

        // Unpack from the correct slot in the 512-bit word
        memcpy(&temp_edge, ((char*)&temp_word) + (slot_idx * sizeof(edge_t)), sizeof(edge_t));

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
    // Setup Buffers and Parameters
    std::cout << "--- C-Simulation Testbench ---" << std::endl;

    /* Read tests from run_list_fpga.txt */
    std::map<std::string, std::vector<TestEntry>> tests;
    std::string prev_datagraph;

    std::ifstream testfile("scripts/run_list_fpga.txt");
    if (!testfile) {
        std::cerr << "Error: Unable to open scripts/run_list_fpga.txt" << std::endl;
        return -1;
    }

    std::string line;
    while (std::getline(testfile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::string datagraph, querygraph, golden, h1, h2;
        std::istringstream iss(line);
        iss >> datagraph >> querygraph >> golden >> h1 >> h2;
        if (datagraph == prev_datagraph) {
            tests[datagraph].push_back({querygraph, golden, h1, h2});
        } else {
            tests[datagraph] = {{querygraph, golden, h1, h2}};
        }
        prev_datagraph = datagraph;
    }
    testfile.close();

    /* Allocate Memory */
    row_t* htb_buf0 = new row_t[HASHTABLES_SPACE];
    row_t* htb_buf1 = new row_t[HASHTABLES_SPACE];
    row_t* htb_buf2 = new row_t[HASHTABLES_SPACE];
    row_t* htb_buf3 = new row_t[HASHTABLES_SPACE];
    row_t* res_buf = new row_t[RESULTS_SPACE];
    row_t* bloom_p = new row_t[BLOOM_SPACE];

    bool all_tests_passed = true;

    /* Loop through all tests from the file */
    for (const auto& datagraph_entry : tests) {
        const std::string& datagraph = datagraph_entry.first;
        const std::vector<TestEntry>& queries = datagraph_entry.second;

        std::cout << "\n================================================================" << std::endl;
        std::cout << "DATAGRAPH: " << datagraph << std::endl;
        std::cout << "================================================================" << std::endl;

        for (const auto& test : queries) {
            std::cout << "\n--- Running Test ---" << std::endl;
            std::cout << "  Query:  " << test.querygraph << std::endl;
            std::cout << "  Golden: " << test.golden << std::endl;

            /* Initialize variables for each test run */
            memset(htb_buf0, 0, HASHTABLES_SPACE * sizeof(row_t));
            memset(htb_buf1, 0, HASHTABLES_SPACE * sizeof(row_t));
            memset(htb_buf2, 0, HASHTABLES_SPACE * sizeof(row_t));
            memset(htb_buf3, 0, HASHTABLES_SPACE * sizeof(row_t));
            memset(res_buf, 0, RESULTS_SPACE * sizeof(row_t));
            memset(bloom_p, 0, BLOOM_SPACE * sizeof(row_t));

            unsigned short nQV = 0;
            unsigned short nQE = 0;
            unsigned short tablelist_length = 0;
            unsigned long nDE = 0;
            unsigned long dynfifo_space = 0;
            long unsigned int golden = std::stoul(test.golden);

            /* Load Graph Data into Buffers */
            // First read query file to get counts for allocation
            std::ifstream fQuery(test.querygraph.substr(3));
            std::string fLine;
            std::getline(fQuery, fLine);
            sscanf(fLine.c_str(), "%*c %hu %hu", &nQV, &nQE);
            fQuery.close();

            unsigned long next_write_p = load_datagraphs<VERTEX_WIDTH_BIT,
                    LABEL_WIDTH,
                    DYN_FIFO_BURST,
                    RESULTS_SPACE,
                    MAX_QUERYDATA>(
                    res_buf, datagraph, dynfifo_space, nDE, nQV, nQE);

            auto res = load_querygraphs<VERTEX_WIDTH_BIT,
                        LABEL_WIDTH,
                        MAX_QUERYDATA>(
                    res_buf, next_write_p, test.querygraph, dynfifo_space, nQV, nQE, nDE, tablelist_length);

            int max_degree = res.second;

            /* Dynamic H1/H2 Calculation */
            unsigned char h1, h2;
            std::cout << "INFO: Using dynamic heuristic for H1/H2." << std::endl;
            h1 = static_cast<unsigned char>(0.4 * log(5e7 * nDE)) + 2;
            h2 = static_cast<unsigned char>(std::min(max_degree + 1, 7));
            if (h1 + h2 <= 14) {
                h2 = 14 - h1; // assert H1+H2 atleast 14
                std::cout << "INFO: Adjusted H2 to " << (int)h2 << " to meet minimum hash width sum requirement." << std::endl;
            }            
            std::cout << "INFO: Using H1=" << (int)h1 << ", H2=" << (int)h2 << std::endl;

            /* Call the HLS Kernel Function */
            long unsigned int result_actual = 0;
            unsigned int dynfifo_overflow = 0;
            volatile unsigned int debug_endpreprocess_s = 0;
            unsigned long p_hits[5] = {0};
            unsigned long p_reqs[7] = {0};

            std::cout << "Starting C Simulation of the kernel..." << std::endl;
            subgraphIsomorphism(
                htb_buf0, htb_buf1, htb_buf2, htb_buf3,
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

            // --- Check Results ---
            std::cout << "Golden Result: " << golden << std::endl;
            std::cout << "Actual Result: " << result_actual << std::endl;
            bool match = (result_actual == golden);
            std::cout << "Test " << (match ? "PASSED" : "FAILED") << std::endl;
            if (!match) {
                all_tests_passed = false;
            }
        }
    }

    /* Clean up memory */
    delete[] htb_buf0;
    delete[] htb_buf1;
    delete[] htb_buf2;
    delete[] htb_buf3;
    delete[] res_buf;
    delete[] bloom_p;

    std::cout << "\n========================================================" << std::endl;
    std::cout << "OVERALL RESULT: " << (all_tests_passed ? "ALL TESTS PASSED" : "SOME TESTS FAILED") << std::endl;
    std::cout << "========================================================" << std::endl;

    return (all_tests_passed ? 0 : 1);
}