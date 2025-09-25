#include "Parameters.hpp"

void scatter_kernel(
    row_t* htb_input,
    row_t* htb_output0,
    row_t* htb_output1,
    row_t* htb_output2,
    row_t* htb_output3
) {

    #pragma HLS INTERFACE mode=m_axi port=htb_input bundle=gmem_in max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0

    #pragma HLS INTERFACE mode=m_axi port=htb_output0 bundle=gmem_out0 max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0
    #pragma HLS INTERFACE mode=m_axi port=htb_output1 bundle=gmem_out1 max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0
    #pragma HLS INTERFACE mode=m_axi port=htb_output2 bundle=gmem_out2 max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0
    #pragma HLS INTERFACE mode=m_axi port=htb_output3 bundle=gmem_out3 max_widen_bitwidth=512 \
    num_read_outstanding=8 num_write_outstanding=8 max_read_burst_length=32 max_write_burst_length=32 latency=0

    #pragma HLS INTERFACE mode=s_axilite port=return

    SCATTER_KERNEL_COPY_LOOP:
    for (int i = 0; i < HASHTABLES_SPACE; ++i) {
        #pragma HLS PIPELINE II=1

        row_t temp_data = htb_input[i];

        htb_output0[i] = temp_data;
        htb_output1[i] = temp_data;
        htb_output2[i] = temp_data;
        htb_output3[i] = temp_data;
    }
}