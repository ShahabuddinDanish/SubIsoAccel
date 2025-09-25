#include "preprocess.hpp"

void preprocess_kernel(
    row_t* edge_buf,
    const unsigned short numQueryVert,
    const unsigned short numQueryEdges,
    const unsigned long numDataEdges,
    const unsigned char hash1_w,
    const unsigned char hash2_w,
    const unsigned long dynfifo_space,
    row_t* htb_output,
    row_t* bloom_p,
    QueryVertex* qVertices_out,
    AdjHT* hTables0_out,
    AdjHT* hTables1_out,
    unsigned int* n_candidate,
    unsigned int* start_candidate
) {
    #pragma HLS INTERFACE mode=m_axi port=edge_buf bundle=gmem_in max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=2 latency=0
    
    #pragma HLS INTERFACE mode=m_axi port=htb_output bundle=gmem_htb_out max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0

    #pragma HLS INTERFACE mode=m_axi port=bloom_p bundle=gmem_bloom_out max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0

    #pragma HLS INTERFACE mode=m_axi port=qVertices_out bundle=gmem_meta_out max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0

    #pragma HLS INTERFACE mode=m_axi port=hTables0_out bundle=gmem_meta_out max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0

    #pragma HLS INTERFACE mode=m_axi port=hTables1_out bundle=gmem_meta_out max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0

    #pragma HLS INTERFACE mode=s_axilite port=numQueryVert
    #pragma HLS INTERFACE mode=s_axilite port=numQueryEdges
    #pragma HLS INTERFACE mode=s_axilite port=numDataEdges
    #pragma HLS INTERFACE mode=s_axilite port=hash1_w
    #pragma HLS INTERFACE mode=s_axilite port=hash2_w
    #pragma HLS INTERFACE mode=s_axilite port=dynfifo_space
    #pragma HLS INTERFACE mode=s_axilite port=n_candidate
    #pragma HLS INTERFACE mode=s_axilite port=start_candidate
    #pragma HLS INTERFACE mode=s_axilite port=return

    QueryVertex qVertices[MAX_QV];
    AdjHT hTables0[MAX_TB], hTables1[MAX_TB];
    unsigned int local_n_candidate;
    unsigned int local_start_candidate;

    preprocess<row_t,
               bloom_t,
               EDGE_WIDTH,
               COUNTER_WIDTH,
               BLOOM_FILTER_WIDTH,
               K_FUNCTIONS,
               DDR_BIT,
               VERTEX_WIDTH_BIT,
               VERTEX_WIDTH,
               HASH_LOOKUP3_BIT,
               MAX_HASH_TABLE_BIT,
               64,
               LABEL_WIDTH,
               DEFAULT_STREAM_DEPTH,
               HASHTABLES_SPACE,
               MAX_QUERY_VERTICES,
               MAX_TABLES,
               MAX_COLLISIONS>(edge_buf,
                               htb_output,
                               htb_output,
                               bloom_p,
                               qVertices,
                               hTables0,
                               hTables1,
                               dynfifo_space,
                               local_n_candidate,
                               local_start_candidate,
                               numQueryVert,
                               numQueryEdges,
                               numDataEdges,
                               hash1_w,
                               hash2_w);

    for(int i = 0; i < MAX_QV; ++i) qVertices_out[i] = qVertices[i];
    for(int i = 0; i < MAX_TB; ++i) hTables0_out[i] = hTables0[i];
    for(int i = 0; i < MAX_TB; ++i) hTables1_out[i] = hTables1[i];

    *n_candidate = local_n_candidate;
    *start_candidate = local_start_candidate;
}


#pragma GCC diagnostic pop
