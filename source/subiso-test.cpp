#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <chrono>
#include <stdexcept>
#include <iomanip>
#include <unistd.h>
#include <fstream>

#if SOFTWARE_PREPROC
    #include "preprocess.hpp"
#endif /* SOFTWARE_PREPROC */

#include "cmdlineparser.h"
#include "xrt/xrt_bo.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_kernel.h"
#include "experimental/xrt_xclbin.h"

// AXI-Lite Register Offsets
const int ADDR_HITS_FINDMIN_LOW = 0xb0;
const int ADDR_HITS_READMIN_C_LOW = 0xc8;
const int ADDR_HITS_READMIN_E_LOW = 0xe0;
const int ADDR_HITS_INTERSECT_LOW = 0xf8;
const int ADDR_HITS_VERIFY_LOW = 0x110;
const int ADDR_REQS_FINDMIN_LOW = 0x128;
const int ADDR_REQS_READMIN_C_LOW = 0x140;
const int ADDR_REQS_READMIN_E_LOW = 0x158;
const int ADDR_REQS_INTERSECT_LOW = 0x170;
const int ADDR_REQS_VERIFY_LOW = 0x188;
const int ADDR_REQS_DYNFO_LOW = 0x1a0;
const int ADDR_BLOOM_FILTERED_LOW = 0x1b8;

const int ADDR_DYNFO_OVERFLOW_DATA = 0x90;
const int ADDR_RESULT_LOW = 0x1d0;
const int ADDR_RESULT_HIGH = 0x1d4;

struct TestEntry {
    std::string querygraph;
    std::string golden;
    std::string h1;
    std::string h2;
};

// Define a 128-bit type for the host
struct host_uint128_t {
    uint64_t low;
    uint64_t high;
};

// Parameter definitions

/* Query graph definitions */
#define VERTEX_WIDTH        5   /* 4 bytes */
#define VERTEX_WIDTH_BIT    (1UL << VERTEX_WIDTH)
#define EDGE_WIDTH          (VERTEX_WIDTH + 1)
#define LABEL_WIDTH         5
#define MAX_QUERYDATA       300

/* Bloom filter parameters */
#define BLOOM_FILTER_WIDTH  7

/* Dynamic fifo parameters */
#define DYN_FIFO_DEPTH      64
#define DYN_FIFO_BURST      32

#define DDR_BIT             7
#define DDR_WORD            (1UL << DDR_BIT)

#define HASHTABLES_SPACE    ((1UL << 28) / (DDR_WORD / 8))  //~ 256 MB
#define BLOOM_SPACE         ((1UL << 27) / (DDR_WORD / 8))  //~ 128 MB
#define RESULTS_SPACE		(DYN_FIFO_BURST * (1UL << 21))  //~ 1 << 30, 1024 MB

typedef host_uint128_t row_t;
typedef host_uint128_t bloom_t;

struct edge_t {
    uint32_t src;
    uint32_t dst;
    uint32_t labelsrc;
    uint32_t labeldst;
};

template <size_t NODE_W,
         size_t LAB_W,
         size_t BURST_SIZE,
         size_t RESULT_SPACE,
         size_t MAX_QDATA>
void load_datagraphs(
        row_t *edge_buf,
        std::string datafile,
        unsigned long &dynfifo_space,
        unsigned long &numDataEdges)
{
    unsigned long numDataVertices;
    unsigned long edge_buf_p = 0;
    edge_t edge;
    
    /* Remove "../" to make paths correct */
    datafile = datafile.substr(3);
    
    std::ifstream fData(datafile);
    
    if (!fData.is_open()){
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
    if (dynfifo_space > RESULT_SPACE){
        std::cout << "Not enough space for dynamic fifo.\n";
        return;
    }
    dynfifo_space = RESULT_SPACE - dynfifo_space;
    edge_buf_p = dynfifo_space;

    /* Store data labels */
    for(int count = 0; count < numDataVertices; count++){    
        unsigned long node_t, label_t;
        std::getline(fData, fLine);
        sscanf(fLine.c_str(), "%*c %lu %lu %*u", &node_t, &label_t);
        vToLabelData.insert(std::make_pair(node_t, label_t));
    }
    
    std::cout << "Loading datagraph in DDR..." << std::endl;
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
        //edge_buf[edge_buf_p++] = *((row_t*)&edge); /*dereferencing type-punned pointer will break strict-aliasing rules warning*/
        memcpy(&edge_buf[edge_buf_p++], &edge, sizeof(row_t));
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Done in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms." << std::endl;

    fData.close();
}

template <size_t NODE_W,
         size_t LAB_W,
         size_t MAX_QDATA>
int load_querygraphs(
        row_t *edge_buf,
        std::string queryfile,
        const unsigned long dynfifo_space,
        unsigned short &numQueryVertices,
        unsigned short &numQueryEdges,
        unsigned short &tableListLength,
        unsigned long numDataEdges)
{
    unsigned long numDataVertices;
    unsigned long edge_buf_p = numDataEdges + dynfifo_space;
    std::string fLine{};
    std::unordered_map<unsigned long, unsigned long> vToLabelQuery;
    std::vector<std::vector<unsigned long>> adjacency_list(numQueryVertices);
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
        return -1;
    }

    /* Read vertices and edges cardinality */
    std::getline(fQuery, fLine);
    sscanf(fLine.c_str(), "%*c %hu %hu", &numQueryVertices, &numQueryEdges);
    std::cout << "Query vertices: " << numQueryVertices << ", Query edges: " << numQueryEdges << std::endl;

    assert(MAX_QDATA >= numQueryEdges + numQueryVertices);
    
    /* Store query labels */
    for (int count = 0; count < numQueryVertices; count++)
    {
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
            return -1;
        }

        query_vertices.erase(remove(query_vertices.begin(), query_vertices.end(), following), query_vertices.end());
        order.push_back(following);
    }

    /* Stream matching order */
    std::cout << "Query vertex order: [";
    for(int count = 0; count < numQueryVertices; count++){ 
        edge.src = order[count];
        std::cout << order[count] << " ";
        //edge_buf[edge_buf_p++] = *((row_t*)&edge); /*dereferencing type-punned pointer will break strict-aliasing rules warning*/
        memcpy(&edge_buf[edge_buf_p++], &edge, sizeof(row_t));
    }
    std::cout << "]" << std::endl;
    
    /* Stream edges */
    for(int count = 0; count < numQueryEdges; count++){    
        unsigned long nodesrc_t, nodedst_t;
        auto tuple_edge = edge_list[count];
        edge.src = std::get<0>(tuple_edge);
        edge.dst = std::get<1>(tuple_edge);
        edge.labelsrc = std::get<2>(tuple_edge);
        edge.labeldst = std::get<3>(tuple_edge);
        //edge_buf[edge_buf_p++] = *((row_t*)&edge); /*dereferencing type-punned pointer will break strict-aliasing rules warning*/
        memcpy(&edge_buf[edge_buf_p++], &edge, sizeof(row_t));
    }

    tableListLength = tablelist.size();
    return 0;

}

int main(int argc, char** argv)
{
    // Command Line Parser
    sda::utils::CmdLineParser parser;

    // Switches
    parser.addSwitch("--xclbin_file", "-x", "input binary file string", "");
    parser.addSwitch("--device_id", "-d", "device index", "0");
    parser.parse(argc, argv);

    // Read settings
    std::string binaryFile = parser.value("xclbin_file");
    int device_index = stoi(parser.value("device_id"));

    if (argc < 3) {
    parser.printHelp();
    return EXIT_FAILURE;
    }

    std::cout << "INFO: Initializing the device " << device_index << std::endl;
    auto device = xrt::device(device_index);
    std::cout << "INFO: Loading the xclbin " << binaryFile << std::endl;
    auto uuid = device.load_xclbin(binaryFile);

    auto krnl = xrt::kernel(device, uuid, "subgraphIsomorphism", xrt::kernel::cu_access_mode::exclusive);

#if COUNT_ONLY
    long unsigned int result;
#else
    //hls::stream<T_NODE> result("results");
#endif

    const unsigned int MAX_QDATA = MAX_QUERYDATA;
    const unsigned int BURST_SIZE = 32;

    unsigned int nfile = 0;
    unsigned int tot_time_bench = 0;
    unsigned int time_limit = 2400;
    unsigned int lab_w = 5;
    unsigned int datagraph_v = 0;
    unsigned int datagraph_e = 0;
    unsigned int querygraph_v = 0;
    unsigned int querygraph_e = 0;
    unsigned int mem_counter = 0;
    unsigned int byte_bloom = 16 * 4;
    unsigned int byte_counter = 4;
    unsigned int byte_edge = 5;

    std::map<std::string, std::vector<TestEntry>> test;
    std::string prev_datagraph;
    unsigned short nQV = 0;
    unsigned short nQE = 0;
    unsigned short tablelist_length = 0;
    unsigned long nDE = 0;
    uint64_t res_actual;
    uint64_t res_expected;
    unsigned long diagnostic;
    unsigned long dynfifo_space;

    // Host-side variables for scalar outputs, kernel won't write to these directly
    unsigned long counters[12] = {0};
    std::string counters_meaning[] = {"hits_findmin",
                                    "hits_readmin_counter",
                                    "hits_readmin_edge",
                                    "hits_intersect",
                                    "hits_verify",
                                    "reqs_findmin",
                                    "reqs_readmin_counter",
                                    "reqs_readmin_edge",
                                    "reqs_intersect",
                                    "reqs_verify",
                                    "reqs_dynfifo",
                                    "bloom_filtered"};
    unsigned int dynfifo_overflow = 0;
    unsigned int debug_endpreprocess_s;
    bool flag = true;

    row_t *res_buf;
    row_t *htb_buf;
    bloom_t *bloom_p;

    char cwd[100];
    if (getcwd(cwd, sizeof(cwd)) != NULL)
        std::cout << "Current working dir: " << cwd << std::endl;

    posix_memalign((void **)&res_buf, 4096, RESULTS_SPACE * sizeof(row_t));
    posix_memalign((void **)&htb_buf, 4096, HASHTABLES_SPACE * sizeof(row_t));
    posix_memalign((void **)&bloom_p, 4096, BLOOM_SPACE * sizeof(bloom_t));

    // row_t *htb_buf = (row_t *)calloc(HASHTABLES_SPACE, sizeof(row_t));
    if (!htb_buf)
    {
        std::cout << "Allocation failed." << std::endl;
        return -1;
    }

    // bloom_t *bloom_p = (bloom_t *)calloc(BLOOM_SPACE, sizeof(bloom_t));
    if (!bloom_p)
    {
        std::cout << "Allocation failed." << std::endl;
        return -1;
    }
    
    // res_buf = (row_t*)malloc(RESULTS_SPACE * sizeof(row_t));
    if (!res_buf){
		std::cout << "Allocation failed." << std::endl;
		return -1;
	}

    std::ifstream testfile("scripts/run_list_fpga.txt");
    if (!testfile) {
        std::cerr << "Error: Unable to open run_list_fpga.txt" << std::endl;
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
            test[datagraph].push_back({querygraph, golden, h1, h2});
        } else {
            test[datagraph] = {{querygraph, golden, h1, h2}};
        }
        prev_datagraph = datagraph;
    }

    testfile.close();

    char datagraph_file[100], querygraph_file[100];

    std::cout << "INFO: Configuring the kernel run object." << std::endl;
    auto run = xrt::run(krnl);

    std::cout << "Creating XRT buffer objects..." << std::endl;

    // Buffer object flags
    auto bo_flags = xrt::bo::flags::normal;

    // Create buffer objects from host-side pointers
    auto htb_buf_bo_b0 = xrt::bo(device, htb_buf, HASHTABLES_SPACE * sizeof(row_t), bo_flags, krnl.group_id(0));    // Argument 0 -> htb_buf -> Bank 0
    auto htb_buf_bo_b1 = xrt::bo(device, htb_buf, HASHTABLES_SPACE * sizeof(row_t), bo_flags, krnl.group_id(1));    // Argument 1 -> htb_buf -> Bank 1
    auto htb_buf_bo_b2 = xrt::bo(device, htb_buf, HASHTABLES_SPACE * sizeof(row_t), bo_flags, krnl.group_id(2));    // Argument 2 -> htb_buf -> Bank 2
    auto htb_buf_bo_b3 = xrt::bo(device, htb_buf, HASHTABLES_SPACE * sizeof(row_t), bo_flags, krnl.group_id(3));    // Argument 3 -> htb_buf -> Bank 3

    auto bloom_bo = xrt::bo(device, bloom_p, BLOOM_SPACE * sizeof(bloom_t), bo_flags, krnl.group_id(0));
    auto res_buf_bo = xrt::bo(device, res_buf, RESULTS_SPACE * sizeof(row_t), bo_flags, krnl.group_id(1));

    for (const auto& entry : test) {
        const std::string& datagraph = entry.first;
        const std::vector<TestEntry>& entries = entry.second;

        std::cout << "Datagraph: " << datagraph << std::endl;

        // load graph
        load_datagraphs<
            VERTEX_WIDTH_BIT,
            LABEL_WIDTH,
            DYN_FIFO_BURST,
            RESULTS_SPACE,
            MAX_QUERYDATA>(
            res_buf,
            std::string(datagraph),
            dynfifo_space,
            nDE);
        
        // need to sync the datagraph buffer once per datagraph
        res_buf_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);

        for (const TestEntry &testEntry : entries)
        {
            // load query
            auto res = load_querygraphs<
                VERTEX_WIDTH_BIT,
                LABEL_WIDTH,
                MAX_QUERYDATA>(
                res_buf,
                std::string(testEntry.querygraph),
                dynfifo_space,
                nQV,
                nQE,
                tablelist_length,
                nDE);
            
            if (res < 0) {
                std::cout << "Error loading query graph." << std::endl;
                return -1;
            }
    
            // query data is written into res_buf, so it needs to be synced again
            res_buf_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);

            std::cout << "  Querygraph: " << testEntry.querygraph << ", Golden: " << testEntry.golden
                      << ", H1: " << testEntry.h1 << ", H2: " << testEntry.h2 << std::endl;
            std::cout << "Words for dynamic fifo: " << dynfifo_space << std::endl;
            unsigned char h1 = stoi(testEntry.h1);
            unsigned char h2 = stoi(testEntry.h2);
            res_expected = stoull(testEntry.golden);

            auto blocks = tablelist_length * 2^(h1 + h2 - 14);
            if (blocks > 4096){
                std::cout << "Error: Blocks overflow." << std::endl;
                return -1;
            }

            auto hashtable_size = tablelist_length * 2^(h1 + h2) * 4;
            hashtable_size += nDE * 16;
            if (hashtable_size > HASHTABLES_SPACE * sizeof(row_t)){
                std::cout << "Error: Hashtable overflow." << std::endl;
                return -1;
            }

            auto bloom_size = tablelist_length * 2^(h1) * 64;
            if (bloom_size > BLOOM_SPACE * sizeof(row_t)){
                std::cout << "Error: Bloom overflow." << std::endl;
                return -1;
            }

            std::cout << "Allocating " << 
                (unsigned long)((HASHTABLES_SPACE + RESULTS_SPACE) * sizeof(row_t) +
                BLOOM_SPACE * sizeof(bloom_t)) << " bytes." << std::endl;
            std::cout << "Hashtables use " << (hashtable_size / (HASHTABLES_SPACE * sizeof(row_t))) * 100 << "% of space." << std::endl; 
            std::cout << "Bloom use " << (bloom_size / (BLOOM_SPACE * sizeof(bloom_t))) * 100 << "% of space." << std::endl;

            //for (int g = 0; g < HASHTABLES_SPACE; htb_buf[g++] = 0);
            //for (int g = 0; g < BLOOM_SPACE; bloom_p[g++] = 0);

            memset(htb_buf, 0, HASHTABLES_SPACE * sizeof(row_t));
            memset(bloom_p, 0, BLOOM_SPACE * sizeof(bloom_t));

            // Sync the buffers that contain input data to the device
            std::cout << "Synchronize input buffer data to device global memory." << std::endl;
            // htb_buf and bloom_p are output-only from the CPU's perspective (written to by the kernel)
            // but the kernel may read from them as well later, so sync. res_buf contains the graph data
            htb_buf_bo_b0.sync(XCL_BO_SYNC_BO_TO_DEVICE);
            htb_buf_bo_b1.sync(XCL_BO_SYNC_BO_TO_DEVICE);
            htb_buf_bo_b2.sync(XCL_BO_SYNC_BO_TO_DEVICE);
            htb_buf_bo_b3.sync(XCL_BO_SYNC_BO_TO_DEVICE);
            bloom_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);

            std::cout << "Setting kernel arguments and launching." << std::endl;

            run.set_arg(0, htb_buf_bo_b0);
            run.set_arg(1, htb_buf_bo_b1);
            run.set_arg(2, htb_buf_bo_b2);
            run.set_arg(3, htb_buf_bo_b3);
            run.set_arg(4, bloom_bo);
            run.set_arg(5, res_buf_bo);
            run.set_arg(6, nQV);
            run.set_arg(7, nQE);
            run.set_arg(8, nDE);
            run.set_arg(9, h1);
            run.set_arg(10, h2);
            run.set_arg(11, dynfifo_space);
            run.set_arg(12, dynfifo_overflow);
#if DEBUG_INTERFACE
            run.set_arg(13, debug_endpreprocess_s);
            run.set_arg(14, counters[0]);
            run.set_arg(15, counters[1]);
            run.set_arg(16, counters[2]);
            run.set_arg(17, counters[3]);
            run.set_arg(18, counters[4]);
            run.set_arg(19, counters[5]);
            run.set_arg(20, counters[6]);
            run.set_arg(21, counters[7]);
            run.set_arg(22, counters[8]);
            run.set_arg(23, counters[9]);
            run.set_arg(24, counters[10]);
            run.set_arg(25, counters[11]);
#endif

            std::cout << "Starting kernel execution." << std::endl;
            auto kernel_start = std::chrono::high_resolution_clock::now();

            run.start();
            std::cout << "Waiting for kernel to complete." << std::endl;
            run.wait();

            auto kernel_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> kernel_time = kernel_end - kernel_start;

            // Get the number of matches from the kernel's registers
            uint32_t res_l = krnl.read_register(ADDR_RESULT_LOW);
            uint32_t res_h = krnl.read_register(ADDR_RESULT_LOW + 4);
            uint64_t total_matches = (static_cast<uint64_t>(res_h) << 32) | res_l;

            // Read 32-bit dynfifo_overflow value
            //dynfifo_overflow = krnl.read_register(ADDR_DYNFO_OVERFLOW_DATA);

#if DEBUG_INTERFACE
            // Read all 64-bit debug counters
            const int counter_addrs[] = {
                ADDR_HITS_FINDMIN_LOW, ADDR_HITS_READMIN_C_LOW, ADDR_HITS_READMIN_E_LOW,
                ADDR_HITS_INTERSECT_LOW, ADDR_HITS_VERIFY_LOW, ADDR_REQS_FINDMIN_LOW,
                ADDR_REQS_READMIN_C_LOW, ADDR_REQS_READMIN_E_LOW, ADDR_REQS_INTERSECT_LOW,
                ADDR_REQS_VERIFY_LOW, ADDR_REQS_DYNFO_LOW, ADDR_BLOOM_FILTERED_LOW
            };
            for (int i = 0; i < 12; ++i) {
                uint32_t low = krnl.read_register(counter_addrs[i]);
                uint32_t high = krnl.read_register(counter_addrs[i] + 4);
                counters[i] = (static_cast<uint64_t>(high) << 32) | low;
            }
#endif

            std::cout << "Expected Matches: " << res_expected << " Actual Matches: " << total_matches << std::endl;
            std::cout << "Execution Time: " << kernel_time.count() << " s" << std::endl;
            /*
            std::cout << "Dynamic FIFO Overflow: " << (dynfifo_overflow ? "YES" : "NO") << std::endl;

            flag &= (res_actual == res_expected);
            std::cout << "Debug counters:" << std::endl;
            for (int g = 0; g < 12; g++) {
                std::cout << "\t" << counters_meaning[g] << ": " << counters[g] << std::endl;
            }
            */
        }
    }
    free(htb_buf);
    free(bloom_p);
    free(res_buf);
    return 0;
    //return !flag;
}
