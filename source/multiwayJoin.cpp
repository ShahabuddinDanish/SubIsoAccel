#include "subgraphIsomorphism.hpp"

void multiwayJoin_kernel(row_t* htb_buf0,
                         row_t* htb_buf1,
                         row_t* htb_buf2,
                         row_t* htb_buf3,
                         row_t* bloom_p,
                         row_t* res_buf, // For the dynamic FIFO
                         QueryVertex qVertices_in[MAX_QV],
                         AdjHT hTables0_in[MAX_TB],
                         AdjHT hTables1_in[MAX_TB],
                         const unsigned short numQueryVert,
                         const unsigned char hash1_w,
                         const unsigned char hash2_w,
                         const unsigned long dynfifo_space,
                         const unsigned int n_candidate,
                         const unsigned int start_candidate,
                         unsigned int &dynfifo_overflow,
#if DEBUG_INTERFACE
                         unsigned long &p_hits_findmin,
                         unsigned long &p_hits_readmin_counter,
                         unsigned long &p_hits_readmin_edge,
                         unsigned long &p_hits_intersect,
                         unsigned long &p_hits_verify,
                         unsigned long &p_reqs_findmin,
                         unsigned long &p_reqs_readmin_counter,
                         unsigned long &p_reqs_readmin_edge,
                         unsigned long &p_reqs_intersect,
                         unsigned long &p_reqs_verify,
                         unsigned long &p_reqs_dynfifo,
                         unsigned long &p_bloom_filtered,
#endif /* DEBUG_INTERFACE */
                         long unsigned int &result
) {
    #pragma HLS INTERFACE mode=m_axi port=htb_buf0 bundle=cache max_widen_bitwidth=512 \
    num_write_outstanding=1 max_write_burst_length=2 latency=0

    #pragma HLS INTERFACE mode=m_axi port=htb_buf1 bundle=readmin_c1 max_widen_bitwidth=512 \
    num_write_outstanding=1 max_read_burst_length=16 max_write_burst_length=2 latency=0
    
    #pragma HLS INTERFACE mode=m_axi port=htb_buf2 bundle=readmin_e max_widen_bitwidth=512 \
    num_write_outstanding=1 max_read_burst_length=16 max_write_burst_length=2 latency=0 

    #pragma HLS INTERFACE mode=m_axi port=htb_buf3 bundle=readmin_c max_widen_bitwidth=512 \
    num_write_outstanding=1 max_read_burst_length=16 max_write_burst_length=2 latency=0

    #pragma HLS INTERFACE mode=m_axi port=bloom_p bundle=bloom max_widen_bitwidth=512 latency=20
    #pragma HLS INTERFACE mode=m_axi port=res_buf bundle=fifo max_widen_bitwidth=512 \
    max_read_burst_length=32 max_write_burst_length=32

    #pragma HLS INTERFACE mode=m_axi port=qVertices_in bundle=gmem_meta max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0
    #pragma HLS INTERFACE mode=m_axi port=hTables0_in bundle=gmem_meta max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0
    #pragma HLS INTERFACE mode=m_axi port=hTables1_in bundle=gmem_meta max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0

    #pragma HLS INTERFACE mode=s_axilite port=numQueryVert
    #pragma HLS INTERFACE mode=s_axilite port=hash1_w
    #pragma HLS INTERFACE mode=s_axilite port=hash2_w
    #pragma HLS INTERFACE mode=s_axilite port=dynfifo_space
    #pragma HLS INTERFACE mode=s_axilite port=n_candidate
    #pragma HLS INTERFACE mode=s_axilite port=start_candidate
    #pragma HLS INTERFACE mode=s_axilite port=dynfifo_overflow
    #pragma HLS INTERFACE mode=s_axilite port=return

#if DEBUG_INTERFACE
    #pragma HLS INTERFACE mode=s_axilite port=p_reqs_findmin
    #pragma HLS INTERFACE mode=s_axilite port=p_reqs_readmin_counter
    #pragma HLS INTERFACE mode=s_axilite port=p_reqs_readmin_edge
    #pragma HLS INTERFACE mode=s_axilite port=p_reqs_intersect
    #pragma HLS INTERFACE mode=s_axilite port=p_reqs_verify
    #pragma HLS INTERFACE mode=s_axilite port=p_reqs_dynfifo
    #pragma HLS INTERFACE mode=s_axilite port=p_hits_findmin
    #pragma HLS INTERFACE mode=s_axilite port=p_hits_readmin_counter
    #pragma HLS INTERFACE mode=s_axilite port=p_hits_readmin_edge
    #pragma HLS INTERFACE mode=s_axilite port=p_hits_intersect
    #pragma HLS INTERFACE mode=s_axilite port=p_hits_verify
    #pragma HLS INTERFACE mode=s_axilite port=p_bloom_filtered
#endif /* DEBUG_INTERFACE */

    #pragma HLS INTERFACE mode=s_axilite port=result

    QueryVertex qVertices[MAX_QV];
    AdjHT hTables0[MAX_TB], hTables1[MAX_TB];

    // Copy metadata from HBM to local BRAM/URAM for fast access
    // This is a small, one-time copy operation
    for(int i = 0; i < MAX_QV; ++i) qVertices[i] = qVertices_in[i];
    for(int i = 0; i < MAX_TB; ++i) hTables0[i] = hTables0_in[i];
    for(int i = 0; i < MAX_TB; ++i) hTables1[i] = hTables1_in[i];

    unsigned long localResult = 0;
    unsigned int local_dynfifo_overflow = 0;
    unsigned long reqs_dynfifo = 0;

    multiwayJoin<bloom_t,
                 BLOOM_FILTER_WIDTH,
                 K_FUNCTIONS,
                 HASH_LOOKUP3_BIT,
                 MAX_HASH_TABLE_BIT,
                 64>(htb_buf0,
                     htb_buf1,
                     htb_buf2,
                     htb_buf3,
                     bloom_p,
                     res_buf,
                     hTables0,
                     hTables1,
                     qVertices,
                     n_candidate,
                     start_candidate,
                     numQueryVert,
                     hash1_w,
                     hash2_w,
                     dynfifo_space,
                     local_dynfifo_overflow,
                     reqs_dynfifo,
                     localResult);
    
    result = localResult;
    dynfifo_overflow = local_dynfifo_overflow;
    p_reqs_findmin = reqs_findmin;
    p_reqs_readmin_counter = reqs_readmin_counter;
    p_reqs_readmin_edge = reqs_readmin_edge;
    p_reqs_intersect = reqs_intersect;
    p_reqs_verify = reqs_verify;
    p_reqs_dynfifo = reqs_dynfifo;
    p_hits_findmin = hits_findmin;
    p_hits_readmin_counter = hits_readmin_counter;
    p_hits_readmin_edge = hits_readmin_edge;
    p_hits_verify = hits_verify;
    p_hits_intersect = hits_intersect;
    p_bloom_filtered = bloom_filtered;

}

#pragma GCC diagnostic pop
