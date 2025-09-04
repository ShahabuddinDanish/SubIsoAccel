#pragma once

#define HLS_STREAM_THREAD_SAFE
#ifndef __SYNTHESIS__
#include <cassert>
#include <fstream>
#include <limits.h>
#endif

#include <hls_stream.h>
#include <ap_int.h>

#include "Parameters.hpp"
#include "QueryVertex.hpp"
#include "Trie.hpp"
#include "hash_lookup3.hpp"

#if DEBUG_STATS
#include "debug.hpp"
#endif /* DEBUG_STATS */

#pragma GCC diagnostic push
// #pragma GCC diagnostic error "-Wpedantic"
// #pragma GCC diagnostic error "-Wall"
// #pragma GCC diagnostic error "-Wextra"
#pragma GCC diagnostic ignored "-Wunused-label"
#pragma GCC diagnostic ignored "-Wsign-compare"
// #pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunknown-pragmas"

struct bloom_write_tuple_t
{
  unsigned int address;
  bool last;
};

template<size_t HASH_W>
struct bloom_update_tuple_t
{
  unsigned int address;
  ap_uint<HASH_W> indexed_h;
  bool write;
  bool last;
};

template<size_t NODE_W>
struct bagtoset_tuple_t
{
  ap_uint<NODE_W> indexing_v;
  bool write;
  bool last;
  bool valid;
};

template<size_t NODE_W>
struct batch_tuple_t
{
  ap_uint<NODE_W> indexing_v;
  bool last;
};

struct counter_tuple_t
{
  unsigned int address;
  bool stop;
};

template<typename PROCESSED_EDGE_T>
struct store_tuple_t
{
  unsigned int address;
  PROCESSED_EDGE_T edge;
  bool stop;
};

/* Builds the table descriptors based on the information
 * from the query graph. */
template<size_t MAX_QV,
         size_t MAX_TB,
         size_t NODE_W,
         size_t LAB_W,
         size_t MAX_LABELS>
void
buildTableDescriptors(row_t* edge_buf,
                      QueryVertex* qVertices,
                      ap_uint<8> labelToTable[MAX_LABELS][MAX_LABELS],
                      unsigned short& numTables,
                      unsigned short numQueryVert,
                      unsigned short numQueryEdge)
{
    /* Translate from id of vertex to position in the order */
    unsigned short fromNumToPos[MAX_QV];
    constexpr size_t SRC_NODE = 0;
    constexpr size_t DST_NODE = 32;
    constexpr size_t LABELSRC_NODE = 64;
    constexpr size_t LABELDST_NODE = 96;

    /* Filling information about query vertices and copying
     * the vertex order needed by multiway join by processing
     * all instructions (vertices and edged). */
    row_t current_word;
    const unsigned int total_instructions = numQueryVert + numQueryEdge;

DESCRIPTOR_LOOP:
    for (unsigned int i = 0; i < total_instructions; ++i) {
#pragma HLS PIPELINE II=1

        // Calculate which 512-bit word and which 128-bit slot this instruction is in
        unsigned int word_idx = i / INSTR_PER_WORD;
        unsigned int slot_idx = i % INSTR_PER_WORD;

        // Only read a new 512-bit word from memory when at the beginning of one
        if (slot_idx == 0) {
            current_word = edge_buf[word_idx];
        }

        // Unpack the 128-bit instruction from its slot in the current 512-bit word
        ap_uint<INSTR_WIDTH> instr = current_word.range(INSTR_WIDTH * (slot_idx + 1) - 1, INSTR_WIDTH * slot_idx);

        /* Creating table descriptors. Logic is split based
         * on whether it's a vertex or an edge instruction */
        if (i < numQueryVert) {
            // This is a vertex instruction (processing the matching order)
            ap_uint<NODE_W> nodesrc = instr.range(SRC_NODE + NODE_W - 1, SRC_NODE);
            fromNumToPos[nodesrc] = i;
        } else {
            // Edge instruction (processing a query edge)
            bool dirEdge = false;
            ap_uint<8> index = 0;
            
            ap_uint<LAB_W> labeldst = instr.range(LABELDST_NODE + LAB_W - 1, LABELDST_NODE);
            ap_uint<LAB_W> labelsrc = instr.range(LABELSRC_NODE + LAB_W - 1, LABELSRC_NODE);
            ap_uint<NODE_W> nodedst = instr.range(DST_NODE + NODE_W - 1, DST_NODE);
            ap_uint<NODE_W> nodesrc = instr.range(SRC_NODE + NODE_W - 1, SRC_NODE);
            unsigned short nodeSrcPos = fromNumToPos[nodesrc];
            unsigned short nodeDstPos = fromNumToPos[nodedst];

            // Direction of the table is used to understand if the
            // source vertex is indexed or indexing the table
            if (nodeSrcPos < nodeDstPos) {
                dirEdge = true;
            }

#ifndef __SYNTHESIS__
            std::cout << (unsigned int)nodesrc << "(" << (int)labelsrc << ")"
                      << " -> " << (unsigned int)nodedst << "(" << (int)labeldst
                      << ")" << std::endl;
#endif

          // Saving the index of the table in the labels matrix
          // which is indexed by [indexing label][indexed label]
          if (dirEdge) {
            index = labelToTable[labelsrc][labeldst];
            if (index == 0) { index = ++numTables; }
            labelToTable[labelsrc][labeldst] = index;
          } else {
            index = labelToTable[labeldst][labelsrc];
            if (index == 0) { index = ++numTables; }
            labelToTable[labeldst][labelsrc] = index;
          }

#ifndef __SYNTHESIS__
          if (dirEdge) {
            std::cout << "Table " << (int)index - 1 << ": " << (int)labelsrc
                      << " -> " << (int)labeldst << std::endl;
          } else {
            std::cout << "Table " << (int)index - 1 << ": " << (int)labeldst
                      << " <- " << (int)labelsrc << std::endl;
          }
#endif

          /* Linking vertices to tables */
          if (dirEdge) {
            unsigned char idx = qVertices[nodeSrcPos].numTablesIndexing;
            qVertices[nodeSrcPos].tables_indexing[idx] = index - 1;
            qVertices[nodeSrcPos].numTablesIndexing++;

            idx = qVertices[nodeDstPos].numTablesIndexed;
            qVertices[nodeDstPos].tables_indexed[idx] = index - 1;
            qVertices[nodeDstPos].vertex_indexing[idx] = nodeSrcPos;

            qVertices[nodeDstPos].numTablesIndexed++;
          } else {
            unsigned char idx = qVertices[nodeDstPos].numTablesIndexing;
            qVertices[nodeDstPos].tables_indexing[idx] = index - 1;
            qVertices[nodeDstPos].numTablesIndexing++;

            idx = qVertices[nodeSrcPos].numTablesIndexed;
            qVertices[nodeSrcPos].tables_indexed[idx] = index - 1;
            qVertices[nodeSrcPos].vertex_indexing[idx] = nodeDstPos;

            qVertices[nodeSrcPos].numTablesIndexed++;
          }
        }
    }
}

/* Reads edges from each table and divide the indexed vertices based
on indexing hash to create bloom filters */
template <typename T_DDR,
          size_t CNT_LOG,
          size_t ROW_LOG,
          size_t NODE_W,
          size_t EDGE_LOG,
          size_t LKP3_HASH_W,
          size_t MAX_HASH_W,
          size_t FULL_HASH_W>
void bloomRead(AdjHT *hTables,
               QueryVertex *qVertices,
               T_DDR *htb_buf,
               const unsigned short numTables,
               const unsigned char hash1_w,
               hls::stream<bagtoset_tuple_t<NODE_W> > &stream_tuple_bagtoset_out,
               hls::stream<bloom_update_tuple_t<FULL_HASH_W>> &stream_tuple_bloom_out) {
  constexpr size_t EDGE_W = 1UL << EDGE_LOG;
  hls::stream<ap_uint<NODE_W>, 4> hash_in0, hash_in1;
  hls::stream<ap_uint<FULL_HASH_W>, 4> hash_out0;
  hls::stream<ap_uint<LKP3_HASH_W>, 4> hash_out1;
  ap_uint<EDGE_W> edge;
  ap_uint<NODE_W> indexing_v, indexed_v, prev_indexing_v;
  ap_uint<FULL_HASH_W> indexed_h, prev_indexed_h;
  ap_uint<MAX_HASH_W> indexing_h, prev_indexing_h;
  bloom_update_tuple_t<FULL_HASH_W> tuple_out;
  T_DDR row;
  unsigned int counter;
  prev_indexing_h = 0;
  prev_indexed_h = 0;

// Select the table with the minimum number of edges
// to start the partial solutions.
  unsigned int minSize = UINT32_MAX;
  unsigned short minTableIndex;

#if DEBUG_PRINTS
  hls::print("\n[bloomRead]: STARTING.\n", 0);
#endif

PROPOSE_TBINDEXING_LOOP:
  for (int g = 0; g < qVertices[0].numTablesIndexing; g++) {
    unsigned short tableIndex = qVertices[0].tables_indexing[g];

    if (hTables[tableIndex].n_edges < minSize) {
      minSize = hTables[tableIndex].n_edges;
      minTableIndex = tableIndex;
    }
  }

BLOOM_READ_TASK_LOOP:
  for (unsigned int ntb = 0; ntb < numTables; ntb++)
  {

    #if DEBUG_PRINTS
      hls::print("[BLOOM_READ_TASK_LOOP]: Processing table %d\n", ntb);
    #endif

    /* During first iteration do not consider the difference between
    prev_indexing_h and indexing_h to be useful to write the bloom */
    bool first_it = true;
    counter = 0;
    unsigned int cycles = (hTables[ntb].n_edges + EDGE_ROW - 1) / EDGE_ROW;
    unsigned int offset = hTables[ntb].start_edges;

    if (hTables[ntb].n_edges == 0) continue; // Skip empty tables

    /* Read all the edges in a table and divide them by hash1 */
  BLOOM_READ_EDGES_BLOCK:
    for (unsigned int start = 0; start < cycles; start++) {
#pragma HLS pipeline II = EDGE_ROW
      row = htb_buf[offset + start];
#if DEBUG_PRINTS
      hls::print("[BLOOM_READ_EDGES_BLOCK]: Reading 512-bit word from htb_buf[%u]\n", (unsigned int)(offset + start));
#endif

      for (int i = 0; i < EDGE_ROW; i++) {
//#pragma HLS unroll
        if ((start * EDGE_ROW + i) < hTables[ntb].n_edges) {
          //edge = row.range(((i + 1) << EDGE_LOG) - 1, i << EDGE_LOG);
          ap_uint<EDGE_W> edge = row;   // Read lowest 64 bits
          row >>= EDGE_W;               // Right-shift for next iteration
          indexing_v = edge.range(NODE_W * 2 - 1, NODE_W);
          indexed_v = edge.range(NODE_W - 1, 0);
#if DEBUG_PRINTS
        hls::print("[BLOOM_READ_EDGES_BLOCK]: Unpacked edge from htb_buf[%d]\n", (unsigned int)(offset + start));
        hls::print("[BLOOM_READ_EDGES_BLOCK]: Unpacked edge from slot %d\n", i);
        hls::print("[BLOOM_READ_EDGES_BLOCK]: Unpacked edge: (%d, ", (unsigned int)indexing_v);
        hls::print("%d)\n", (unsigned int)indexed_v);
#endif
          hash_in0.write(indexed_v);
          hash_in1.write(indexing_v);
          xf::database::hashLookup3<NODE_W>(hash_in0, hash_out0);
          xf::database::hashLookup3<NODE_W>(hash_in1, hash_out1);
          indexed_h = hash_out0.read();
          indexing_h = hash_out1.read();
          indexing_h = indexing_h.range(hash1_w - 1, 0);

          bool valid = (counter < hTables[ntb].n_edges);
          bool write = (indexing_h != prev_indexing_h);
          /* Writing edge of previous iteration */
          if (valid && !first_it) {
            tuple_out.address = ntb * (1UL << hash1_w) + prev_indexing_h;
            tuple_out.last = false;
            tuple_out.write = write;
            tuple_out.indexed_h = prev_indexed_h;
#ifdef DEBUG_PRINTS
            hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE (edge of previous iteration), address=%d\n", (unsigned int)tuple_out.address);
            hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE (edge of previous iteration), indexed_h=%d\n", (unsigned int)tuple_out.indexed_h);
            hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE (edge of previous iteration), write=%d\n", (unsigned int)tuple_out.write);
            hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE (edge of previous iteration), last=%d\n", (unsigned int)tuple_out.last);
#endif
            stream_tuple_bloom_out.write(tuple_out);

            if (ntb == minTableIndex) {
              bagtoset_tuple_t<NODE_W> tuple_bagtoset_out;
              tuple_bagtoset_out.indexing_v = prev_indexing_v;
              tuple_bagtoset_out.write = write;
              tuple_bagtoset_out.last = false;
              tuple_bagtoset_out.valid = true;
#ifdef DEBUG_PRINTS
              hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE 1, indexing_v=%d\n", (unsigned int)tuple_bagtoset_out.indexing_v);
              hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE 1, write=%d\n", (unsigned int)tuple_bagtoset_out.write);
              hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE 1, valid=%d\n",(unsigned int)tuple_bagtoset_out.valid);
              hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE 1, last=%d\n", (unsigned int)tuple_bagtoset_out.last);
#endif
              stream_tuple_bagtoset_out.write(tuple_bagtoset_out);
            }
          }

          if (valid) {
            prev_indexing_h = indexing_h;
            prev_indexing_v = indexing_v;
            prev_indexed_h = indexed_h;
          }
          counter++;
          first_it = false;
        }
      }
    }

    /* Write explicitly the last bloom filter since
    the difference between prev_indexing_h and indexing_h
    does not work at the end of the table */
    tuple_out.address = ntb * (1UL << hash1_w) + prev_indexing_h;
    tuple_out.indexed_h = prev_indexed_h;
    tuple_out.write = true;
    tuple_out.last = (ntb == (numTables - 1));
#ifdef DEBUG_PRINTS
          hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE (last bloom filter), address=%d\n", (unsigned int)tuple_out.address);
          hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE (last bloom filter), indexed_h=%d\n", (unsigned int)tuple_out.indexed_h);
          hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE (last bloom filter), write=%d\n", (unsigned int)tuple_out.write);
          hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE (last bloom filter), last=%d\n", (unsigned int)tuple_out.last);
#endif
    stream_tuple_bloom_out.write(tuple_out);

    bagtoset_tuple_t<NODE_W> tuple_bagtoset_out;
    tuple_bagtoset_out.indexing_v = prev_indexing_v;
    tuple_bagtoset_out.write = true;
    tuple_bagtoset_out.valid = ntb == minTableIndex;
    tuple_bagtoset_out.last = (ntb == (numTables - 1));
#ifdef DEBUG_PRINTS
    hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE 2, indexing_v=%d\n", (unsigned int)tuple_bagtoset_out.indexing_v);
    hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE 2, write=%d\n", (unsigned int)tuple_bagtoset_out.write);
    hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE 2, valid=%d\n", (unsigned int)tuple_bagtoset_out.valid);
    hls::print("[BLOOM_READ_EDGES_BLOCK]: Sending TUPLE 2, last=%d\n", (unsigned int)tuple_bagtoset_out.last);
#endif
    stream_tuple_bagtoset_out.write(tuple_bagtoset_out);
  }
#if DEBUG_PRINTS
  hls::print("[bloomRead]: FINISHED.\n", 0);
#endif
}

template <size_t NODE_W,
          size_t MAX_CL>
void bagtoset(hls::stream<bagtoset_tuple_t<NODE_W> > &stream_tuple_in,
              hls::stream<batch_tuple_t<NODE_W> > &stream_tuple_out)
{
  bagtoset_tuple_t<NODE_W> tuple_in;
  ap_uint<NODE_W> set[MAX_CL];
  unsigned char pointer = 0;
  ap_uint<MAX_CL> valid_bits = 0; // 1 if the element is present
  ap_uint<MAX_CL> equal_bits = 0; // 1 if the element is equal to the one in the bag

#if DEBUG_PRINTS
  hls::print("\n[bagtoset]: STARTING.\n", 0);
#endif

BAGTOSET_TASK_LOOP:
  do {
#pragma HLS pipeline II = 2
#if DEBUG_PRINTS
    hls::print("[BAGTOSET_TASK_LOOP]: Waiting for a tuple from stream.\n", 0);
#endif
    tuple_in = stream_tuple_in.read();
#if DEBUG_PRINTS
    hls::print("[BAGTOSET_TASK_LOOP]: Read tuple, v=%d\n", (unsigned int)tuple_in.indexing_v); 
    hls::print("[BAGTOSET_TASK_LOOP]: Read tuple, valid=%d\n", (int)tuple_in.valid);
    hls::print("[BAGTOSET_TASK_LOOP]: Read tuple, write=%d\n", (int)tuple_in.write);
    hls::print("[BAGTOSET_TASK_LOOP]: Read tuple, last=%d\n", (int)tuple_in.last);
#endif

    if (tuple_in.valid)
    {
      // Check if the element is present in the set
      for (int i = 0; i < MAX_CL; i++)
      {
#pragma HLS unroll
        equal_bits[i] = (set[i] == tuple_in.indexing_v);
      }

      // If the element is not present in the set, add it
      // and write it to the output stream
      if ((equal_bits & valid_bits) == 0) {
        set[pointer] = tuple_in.indexing_v;
        valid_bits = valid_bits | (1 << pointer);
        pointer++;
        batch_tuple_t<NODE_W> tuple_out;
        tuple_out.indexing_v = tuple_in.indexing_v;
        tuple_out.last = false;
#if DEBUG_PRINTS
        hls::print("[BAGTOSET_TASK_LOOP]: Vertex %d is Unique. Writing downstream.\n", (unsigned int)tuple_in.indexing_v);
#endif
        stream_tuple_out.write(tuple_out);
      }
#if DEBUG_PRINTS
      else {
        hls::print("[BAGTOSET_TASK_LOOP]: Vertex %d is a Duplicate. Discarding.\n", (unsigned int)tuple_in.indexing_v);
      }
#endif

      // Last element of the hash set
      if (tuple_in.write)
      {
#if DEBUG_PRINTS
        hls::print("[BAGTOSET_TASK_LOOP]: Received 'write' flag. Resetting uniqueness filter.\n", 0);
#endif
        pointer = 0;
        valid_bits = 0;
      }
    }

  } while (!tuple_in.last);

  batch_tuple_t<NODE_W> tuple_out;
  tuple_out.indexing_v = tuple_in.indexing_v;
  tuple_out.last = true;
#if DEBUG_PRINTS
    hls::print("[bagtoset]: Forwarding final STOP signal downstream.\n", 0);
#endif
  stream_tuple_out.write(tuple_out);
#if DEBUG_PRINTS
  hls::print("[bagtoset]: FINISHED.\n", 0);
#endif
}

template <size_t NODE_LOG,
          size_t NODE_W,
          size_t ROW_LOG>
void batch(unsigned int &n_candidate,
               const unsigned int start_address,
               row_t *htb_buf,
               hls::stream<batch_tuple_t<NODE_W>> &stream_tuple_in)
{
  constexpr size_t NODE_PER_WORD_LOG = ROW_LOG - NODE_LOG;
  row_t word;
  ap_uint<32> pointer = 0;
  unsigned int offset = 0;

#if DEBUG_PRINTS
  hls::print("\n[batch]: STARTING.\n", 0);
  hls::print("[batch]: Writing candidate list starting at htb_buf[%d].\n", start_address);
  hls::print("[batch]: Waiting to read first tuple from stream.\n", 0);
#endif
  batch_tuple_t<NODE_W> tuple_in = stream_tuple_in.read();
#if DEBUG_PRINTS
  hls::print("[batch]: Read first tuple: { v=%d, ", (unsigned int)tuple_in.indexing_v);
  hls::print("last=%d }.\n", (int)tuple_in.last);
#endif

  while (!tuple_in.last)
  {
#pragma HLS pipeline II = 1
#if DEBUG_PRINTS
    hls::print("[batch loop]: Processing candidate vertex: = %d\n", (unsigned int)tuple_in.indexing_v);
#endif
    ap_uint<NODE_PER_WORD_LOG> in_word_pointer = pointer.range(NODE_PER_WORD_LOG - 1, 0);
    word.range(NODE_W * (in_word_pointer + 1) - 1, NODE_W * in_word_pointer) = tuple_in.indexing_v;
    if (in_word_pointer == (1UL << NODE_PER_WORD_LOG) - 1)
    {
#if DEBUG_PRINTS
      hls::print("[batch loop]: Word is full. Writing to htb_buf[%d].\n", (unsigned int)(start_address + offset));
      hls::print("[batch loop]: Word is full. Content (hex): %s\n", word.to_string(16).c_str());
      hls::print("[batch loop]: Content (hex) [511:256]: %s\n", word.range(511, 256).to_string(16).c_str());
      hls::print("[batch loop]: Content (hex) [255:  0]: %s\n", word.range(255,   0).to_string(16).c_str());
#endif
      htb_buf[start_address + offset] = word;
      offset++;
    }
    pointer++;
#if DEBUG_PRINTS
    hls::print("[batch loop]: Waiting to read next tuple from stream.\n", 0);
#endif
    tuple_in = stream_tuple_in.read();
#if DEBUG_PRINTS
  hls::print("[batch loop]: Read next tuple: { v=%d, ", (unsigned int)tuple_in.indexing_v);
  hls::print("last=%d }.\n", (int)tuple_in.last);
#endif
  };
#if DEBUG_PRINTS
  hls::print("[batch]: Loop finished. Writing final word to htb_buf[%d].\n", (unsigned int)(start_address + offset));
  hls::print("[batch]: Content (hex): %s\n", word.to_string(16).c_str());
#endif
  htb_buf[start_address + offset] = word;
  n_candidate = pointer;
#if DEBUG_PRINTS
    hls::print("[batch] FINISHED. Total candidates (n_candidate) = %u\n", (unsigned int)n_candidate);
#endif
}

template<typename T_BLOOM,
         size_t BLOOM_LOG,
         size_t K_FUN_LOG,
         size_t FULL_HASH_W>
void bloomUpdate(hls::stream<bloom_update_tuple_t<FULL_HASH_W> >& stream_tuple_in,
           hls::stream<bloom_write_tuple_t>& stream_address,
           hls::stream<T_BLOOM> stream_filter[(1UL << K_FUN_LOG)])
{
    constexpr size_t K_FUN = (1UL << K_FUN_LOG);
    T_BLOOM filter[K_FUN];
    bloom_update_tuple_t<FULL_HASH_W> tuple_in;
#pragma HLS array_partition variable = filter type = complete

    /*Reset filter*/
    for (auto g = 0; g < K_FUN; g++) {
#pragma HLS unroll
        filter[g] = 0;
    }

    do {
#pragma HLS pipeline II = 1

        tuple_in = stream_tuple_in.read();
        for (int g = 0; g < K_FUN; g++) {
#pragma HLS unroll
            ap_uint<BLOOM_LOG> idx = tuple_in.indexed_h.range(
              (FULL_HASH_W / K_FUN) * (g + 1) - 1,
              (FULL_HASH_W / K_FUN) * (g + 1) - BLOOM_LOG);
            filter[g][idx] = 1;
            if (tuple_in.write) {
                stream_filter[g].write(filter[g]);
                filter[g] = 0;
            }
        }

        if (tuple_in.write) {
            stream_address.write({ tuple_in.address, tuple_in.last });
        }
    } while (!tuple_in.last);
}

template <typename T_BLOOM, size_t K_FUN_LOG>
void bloomWrite(row_t *bloom_p,
                hls::stream<bloom_write_tuple_t> &stream_address,
                hls::stream<T_BLOOM> stream_filter[(1UL << K_FUN_LOG)])
{
  constexpr size_t K_FUN = (1UL << K_FUN_LOG);
  row_t packing_buffer;
  bloom_write_tuple_t tuple_in;

#if DEBUG_PRINTS
  hls::print("\n[bloomWrite]: STARTING.\n", 0);
#endif

BLOOM_WRITE_TASK_LOOP:
  do {
#pragma HLS pipeline II = (1UL << K_FUN_LOG)
#if DEBUG_PRINTS
    hls::print("[BLOOM_WRITE_TASK_LOOP]: Waiting for address tuple from stream.\n", 0);
#endif
    tuple_in = stream_address.read();
#if DEBUG_PRINTS
    hls::print("[BLOOM_WRITE_TASK_LOOP]: Read address tuple, address=%d\n", (unsigned int)tuple_in.address);
    hls::print("[BLOOM_WRITE_TASK_LOOP]: Read address tuple, last=%d\n", (int)tuple_in.last);
#endif

    // Pack the K_FUN filters into a single 512-bit word
    for (int g = 0; g < K_FUN; g++) {
#pragma HLS unroll
      T_BLOOM filter_chunk = stream_filter[g].read();
#if DEBUG_PRINTS
      hls::print("[BLOOM_WRITE_TASK_LOOP]: Packing filter %d\n", g);
      hls::print("[BLOOM_WRITE_TASK_LOOP]: Filter Content (hex): %s\n", filter_chunk.to_string(16).c_str());
#endif
      // Pack the g-th 128-bit filter into the g-th slot of the 512-bit word.
      packing_buffer.range(INSTR_WIDTH * (g + 1) - 1, INSTR_WIDTH * g) = filter_chunk;
    }

#if DEBUG_PRINTS
      hls::print("[BLOOM_WRITE_TASK_LOOP]: Writing packed filter to bloom_p[%d].\n", (unsigned int)tuple_in.address);
      hls::print("[BLOOM_WRITE_TASK_LOOP]: Packing Buffer Content (hex): %s\n", packing_buffer.to_string(16).c_str());
      hls::print("[BLOOM_WRITE_TASK_LOOP]: Packing Buffer Content [511:256]: %s\n", packing_buffer.range(511, 256).to_string(16).c_str());
      hls::print("[BLOOM_WRITE_TASK_LOOP]: Packing Buffer Content [255:  0]: %s\n", packing_buffer.range(255,   0).to_string(16).c_str());
#endif

    // The 512-bit buffer is now full, write packed 512-bit word to DDR
    bloom_p[tuple_in.address] = packing_buffer;

// #if DEBUG_STATS
//     /* Computing the number of ones in each filter*/
//     T_BLOOM row = bloom_p[(tuple_in.address << K_FUN_LOG) + g];
//     while (row > 0) {
//       debug::bloom_fullness++;
//       row = row & (row - 1);
//     }
// #endif /* DEBUG_STATS */

  } while (!tuple_in.last);

#if DEBUG_PRINTS
    hls::print("[bloomWrite]: FINISHED.\n", 0);
#endif
}

template <typename T_DDR,
          typename T_BLOOM,
          size_t CNT_LOG,
          size_t ROW_LOG,
          size_t NODE_W,
          size_t NODE_LOG,
          size_t EDGE_LOG,
          size_t MAX_CL,
          size_t LKP3_HASH_W,
          size_t MAX_HASH_W,
          size_t FULL_HASH_W,
          size_t BLOOM_LOG,
          size_t K_FUN_LOG,
          size_t STREAM_D>
void writeBloom(
    T_DDR *bloom_p,
    T_DDR *htb_p0,
    T_DDR *htb_p1,
    AdjHT *hTables,
    QueryVertex *qVertices,
    unsigned int &n_candidate,
    const unsigned int start_address,
    const unsigned char numTables,
    const unsigned char hash1_w)
{
#pragma HLS DATAFLOW

    hls::stream<bloom_update_tuple_t<FULL_HASH_W>, 32>
      stream_tuple("Bloom tuple");
    hls::stream<bloom_write_tuple_t, 8> stream_address(
      "Bloom address");
    hls::stream<T_BLOOM, 8> stream_filter[(1UL << K_FUN_LOG)];
    hls::stream<bagtoset_tuple_t<NODE_W>, 8> stream_tuple_bagtoset(
      "Bagtoset tuple");
    hls::stream<batch_tuple_t<NODE_W>, 8> stream_tuple_batch(
      "Batch tuple");

    /* Read edges in each table */
    bloomRead<T_DDR,
              CNT_LOG,
              ROW_LOG,
              NODE_W,
              EDGE_LOG,
              LKP3_HASH_W,
              MAX_HASH_W,
              FULL_HASH_W>(hTables,
                           qVertices,
                           htb_p0,
                           numTables,
                           hash1_w,
                           stream_tuple_bagtoset,
                           stream_tuple);

    bloomUpdate<T_BLOOM, BLOOM_LOG, K_FUN_LOG, FULL_HASH_W>(
      stream_tuple, stream_address, stream_filter);

    bagtoset<NODE_W, MAX_CL>(stream_tuple_bagtoset, stream_tuple_batch);

    bloomWrite<T_BLOOM, K_FUN_LOG>(bloom_p, stream_address, stream_filter);

    batch<NODE_LOG, NODE_W, ROW_LOG>(n_candidate, start_address, htb_p1, stream_tuple_batch);
}

template<size_t NODE_W,
         size_t LAB_W,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t MAX_LABELS>
void readEdgesPerBlock(row_t* edge_buf,
                  const unsigned char hash1_w,
                  const unsigned char hash2_w,
                  const ap_uint<8> labelToTable[MAX_LABELS][MAX_LABELS],
                  const unsigned long numDataEdges,
                  hls::stream<counter_tuple_t> stream_address[2])
{
    constexpr size_t COUNTERS_PER_BLOCK = 14;
    constexpr size_t SRC_NODE = 0;
    constexpr size_t DST_NODE = 32;
    constexpr size_t LABELSRC_NODE = 64;
    constexpr size_t LABELDST_NODE = 96;
    const unsigned int block_per_table = hash1_w + hash2_w - COUNTERS_PER_BLOCK;
    const unsigned long num_data_words = (numDataEdges + INSTR_PER_WORD - 1) / INSTR_PER_WORD;
    bool inverted = false;

#if DEBUG_PRINTS
    hls::print("\n[readEdgesPerBlock] STARTING. Will read %d words.\n", (unsigned int)num_data_words);
#endif

READ_EDGES_PER_BLOCK_LOOP:
    for (auto s_word = 0; s_word < num_data_words; s_word++) {
#pragma HLS pipeline II = INSTR_PER_WORD
#if DEBUG_PRINTS
      hls::print("[READ_EDGES_PER_BLOCK_LOOP]: Reading data from res_buf[%d]\n", (unsigned int)(s_word));
      hls::print("[READ_EDGES_PER_BLOCK_LOOP]: Reading 512-bit memory word %d\n", (unsigned int)s_word);
#endif

      row_t packed_edge = edge_buf[s_word]; // Read one 512-bit word

      // Inner loop unpacks 128-bit instructions from the word
      for (int s_unpack = 0; s_unpack < INSTR_PER_WORD; s_unpack++) {
#pragma HLS unroll
        // Boundary check to avoid processing padding data in the last word
        if ((s_word * INSTR_PER_WORD + s_unpack) < numDataEdges) {

          // Extract the 128-bit logical instruction
          ap_uint<INSTR_WIDTH> edge = packed_edge.range(INSTR_WIDTH * (s_unpack + 1) - 1, INSTR_WIDTH * s_unpack);

          ap_uint<LAB_W> labeldst = edge.range(LABELDST_NODE + LAB_W - 1, LABELDST_NODE);
          ap_uint<LAB_W> labelsrc = edge.range(LABELSRC_NODE + LAB_W - 1, LABELSRC_NODE);
          ap_uint<NODE_W> nodedst = edge.range(DST_NODE + NODE_W - 1, DST_NODE);
          ap_uint<NODE_W> nodesrc = edge.range(SRC_NODE + NODE_W - 1, SRC_NODE);
#if DEBUG_PRINTS
        hls::print("[READ_EDGES_PER_BLOCK_LOOP] Read data edge (%d, ", (unsigned int)nodesrc);
        hls::print("%d)\n", (unsigned int)nodedst);
        hls::print("[READ_EDGES_PER_BLOCK_LOOP] Read data edge from res_buf word slot[%d]\n", (unsigned int)(s_unpack));
#endif

          // Retrieve index of table with source as indexing vertex
          ap_uint<8> index0 = labelToTable[labelsrc][labeldst];
          // Retrieve index of table with destination as indexing vertex
          ap_uint<8> index1 = labelToTable[labeldst][labelsrc];

          /* Compute indices for hash table */
          ap_uint<LKP3_HASH_W> hash_out0;
          ap_uint<LKP3_HASH_W> hash_out1;

          xf::database::details::hashlookup3_core<NODE_W>(nodesrc, hash_out0);
          ap_uint<MAX_HASH_W> hashsrc = hash_out0.range(MAX_HASH_W - 1, 0);
          hashsrc = hashsrc.range(hash1_w - 1, 0);

          xf::database::details::hashlookup3_core<NODE_W>(nodedst, hash_out1);
          ap_uint<MAX_HASH_W> hashdst = hash_out1.range(MAX_HASH_W - 1, 0);
          hashdst = hashdst.range(hash1_w - 1, 0);

          /* Compute inside which block the edge will finish, in the meanwhile
          also adjust the edge as indexing -> indexed, and also add the hash
          values */
          unsigned short address_intable0 = hashsrc.range(hash1_w - 1, hash1_w - block_per_table);
          unsigned int address0 = ((index0 - 1) << block_per_table) + address_intable0;
          unsigned short address_intable1 = hashdst.range(hash1_w - 1, hash1_w - block_per_table);
          unsigned int address1 = ((index1 - 1) << block_per_table) + address_intable1;

          /* This useless if is to explain to Vitis HLS 2022.2 that two write in
          * the same stream cannot happen in one cycle */
          if (index0 != 0 && index1 != 0) {
#if DEBUG_PRINTS
            hls::print("    -> Edge matches query in both directions. inverted=%d\n", (int)inverted);
#endif
            if (inverted){
#if DEBUG_PRINTS
              hls::print("        Writing address %d to stream[1]\n", (unsigned int)address0);
#endif
              stream_address[1].write({ address0, false });
#if DEBUG_PRINTS
              hls::print("        Writing address %d to stream[0]\n", (unsigned int)address1);
#endif
              stream_address[0].write({ address1, false });
            } else {
#if DEBUG_PRINTS
              hls::print("        Writing address %d to stream[0]\n", (unsigned int)address0);
#endif
              stream_address[0].write({ address0, false });
#if DEBUG_PRINTS
              hls::print("        Writing address %d to stream[1]\n", (unsigned int)address1);
#endif
              stream_address[1].write({ address1, false });
            }
          } else if (index0 != 0){
#if DEBUG_PRINTS
            hls::print("    -> Edge matches query in forward direction. inverted=%d\n", (int)inverted);
#endif
            if (inverted){
#if DEBUG_PRINTS
              hls::print("        Writing address %d to stream[1]\n", (unsigned int)address0);
#endif
              stream_address[1].write({ address0, false });
            } else {
#if DEBUG_PRINTS
              hls::print("        Writing address %d to stream[0]\n", (unsigned int)address0);
#endif
              stream_address[0].write({ address0, false });
            }
          } else if (index1 != 0){
#if DEBUG_PRINTS
            hls::print("    -> Edge matches query in reverse direction. inverted=%d\n", (int)inverted);
#endif
            if (inverted){
#if DEBUG_PRINTS
              hls::print("        Writing address %d to stream[1]\n", (unsigned int)address1);
#endif
              stream_address[1].write({ address1, false });
            } else {
#if DEBUG_PRINTS
              hls::print("        Writing address %d to stream[0]\n", (unsigned int)address1);
#endif
              stream_address[0].write({ address1, false });
            }
          }

          if ((index0 != 0) ^ (index1 != 0)){
            inverted = !inverted;
          }
        }
      }
    }

/*
    if (inverted){
#if DEBUG_STATS
      hls::print("[readEdgesPerBlock] FINISHED: Sending STOP signal to stream[1]\n");
#endif
      stream_address[1].write({ 0, true });
    } else {
#if DEBUG_STATS
      hls::print("[readEdgesPerBlock] FINISHED: Sending STOP signal to stream[0]\n");
#endif
      stream_address[0].write({ 0, true });
    }
*/
  // Send a stop signal to BOTH consumer streams to prevent deadlock
  hls::print("[readEdgesPerBlock] FINISHED: Sending STOP signal to stream[0]\n");
  stream_address[0].write({0, true});
  hls::print("[readEdgesPerBlock] FINISHED: Sending STOP signal to stream[1]\n");
  stream_address[1].write({0, true});
}

template<size_t NODE_W,
         size_t LAB_W,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t MAX_LABELS>
void
countEdgesPerBlock(hls::stream<counter_tuple_t> stream_address[2],
                  unsigned int block_n_edges[4096])
{
    const size_t BRAM_LAT = 3;
    unsigned int local_cache_address[BRAM_LAT];
    unsigned int local_cache_counter[BRAM_LAT];
    ap_uint<BRAM_LAT> local_cache_valid = 0;
    int stopped_streams = 0;

#if DEBUG_PRINTS
    hls::print("\n[countEdgesPerBlock]: STARTING.\n", 0);
#endif

COUNT_EDGES_PER_BLOCK_LOOP:
    while (stopped_streams < 2) {
#pragma HLS dependence variable = block_n_edges type = inter direction =       \
  RAW false
#pragma HLS pipeline II = 1

        /* On each cycle, try to read from both streams */
        for (int i = 0; i < 2; i++) {
#pragma HLS unroll
            counter_tuple_t tuple_in;
            if (stream_address[i].read_nb(tuple_in)) {
#if DEBUG_PRINTS
                hls::print("[COUNT_EDGES_PER_BLOCK_LOOP]: Read from stream[%d]\n", i);
                hls::print("[COUNT_EDGES_PER_BLOCK_LOOP]: Read addr=%u\n", (unsigned int)tuple_in.address);
                hls::print("[COUNT_EDGES_PER_BLOCK_LOOP]: Read stop=%d\n", (int)tuple_in.stop);
#endif
                if (tuple_in.stop) {
                    stopped_streams++;
#if DEBUG_PRINTS
                    hls::print("[COUNT_EDGES_PER_BLOCK_LOOP]: STOP signal received from stream[%d]\n", i);
                    hls::print("[COUNT_EDGES_PER_BLOCK_LOOP]: Total stopped: %d\n", stopped_streams);
#endif
                } else {
                    unsigned int address = tuple_in.address;
#if DEBUG_PRINTS
                    hls::print("[COUNT_EDGES_PER_BLOCK_LOOP]: Processing address %d\n", (unsigned int)address);
#endif
                    bool hit = false;
                    unsigned int local_value_counter = 0;

                    /* Check if the counter has been used recently, by cycling backword to
                    * catch the updated value */
                    for (auto s = 0; s < BRAM_LAT; s++) {
#pragma HLS unroll
                        auto g = BRAM_LAT - s - 1;
                        if (local_cache_address[g] == address && local_cache_valid[g]) {
                            hit = true;
                            local_value_counter = local_cache_counter[g];
                        }
                    }

                    /* Read from memory only if is not present in local cache, in this way
                    * it is possible to remove the RAW dependency */
                    if (hit){
                      local_value_counter++;
                    } else {
                      local_value_counter = block_n_edges[address];
                      local_value_counter++;
                    }

                    /* Shift everything by one position and writes the last one in memory */
                    for (auto s = 0; s < BRAM_LAT - 1; s++) {
#pragma HLS unroll
                      auto g = BRAM_LAT - s - 1;
                      local_cache_address[g] = local_cache_address[g - 1];
                      local_cache_counter[g] = local_cache_counter[g - 1];
                      local_cache_valid[g] = local_cache_valid[g - 1];
                    }
                    local_cache_address[0] = address;
                    local_cache_counter[0] = local_value_counter;
                    local_cache_valid[0] = true;
                    block_n_edges[address] = local_value_counter;
#ifndef __SYNTHESIS__
                    assert(local_value_counter < UINT32_MAX);
#endif
                  }
            }
        }
    }

#if DEBUG_PRINTS
    hls::print("[countEdgesPerBlock]: FINISHED. Both streams stopped.\n", 0);
#endif
}

template<size_t NODE_W,
         size_t LAB_W,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t MAX_LABELS>
void
countEdgesPerBlockWrap(row_t* edge_buf,
                const unsigned char hash1_w,
                const unsigned char hash2_w,
                const ap_uint<8> labelToTable[MAX_LABELS][MAX_LABELS],
                const unsigned long numDataEdges,
                unsigned int block_n_edges[4096])
{
#pragma HLS dataflow
    hls::stream<counter_tuple_t, 32> stream_address[2];

    readEdgesPerBlock<NODE_W, LAB_W, LKP3_HASH_W, MAX_HASH_W, MAX_LABELS>(
      edge_buf, hash1_w, hash2_w, labelToTable, numDataEdges, stream_address);

    countEdgesPerBlock<NODE_W, LAB_W, LKP3_HASH_W, MAX_HASH_W, MAX_LABELS>(
      stream_address, block_n_edges);
}

template<size_t NODE_W,
         size_t LAB_W,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t MAX_LABELS>
void
readAndStreamEdgesPerBlock(row_t* edge_buf,
                  const unsigned char hash1_w,
                  const unsigned char hash2_w,
                  const ap_uint<8> labelToTable[MAX_LABELS][MAX_LABELS],
                  const unsigned int numDataEdges,
                  hls::stream<store_tuple_t<processed_edge_t> > stream_edge[2])
{
    constexpr size_t COUNTERS_PER_BLOCK = 14;
    constexpr size_t SRC_NODE = 0;
    constexpr size_t DST_NODE = 32;
    constexpr size_t LABELSRC_NODE = 64;
    constexpr size_t LABELDST_NODE = 96;
    const unsigned int block_per_table = hash1_w + hash2_w - COUNTERS_PER_BLOCK;
    const unsigned long num_data_words = (numDataEdges + INSTR_PER_WORD - 1) / INSTR_PER_WORD;
    bool inverted = false;

STORE_EDGE_PER_BLOCK_LOOP:
    // Outer loop iterates over 512-bit memory words
    for (auto s_word = 0; s_word < num_data_words; s_word++) {
#pragma HLS pipeline II = INSTR_PER_WORD

        row_t packed_edge = edge_buf[s_word]; // Read one 512-bit word

        // Inner loop unpacks 128-bit instructions from the memory word
        for (int s_unpack = 0; s_unpack < INSTR_PER_WORD; s_unpack++) {
#pragma HLS unroll

          // Boundary check to avoid processing padding data
          if ((s_word * INSTR_PER_WORD + s_unpack) < numDataEdges) {

            // Extract the 128-bit logical instruction
            ap_uint<INSTR_WIDTH> edge = packed_edge.range(INSTR_WIDTH * (s_unpack + 1) - 1, INSTR_WIDTH * s_unpack);

            ap_uint<LAB_W> labeldst = edge.range(LABELDST_NODE + LAB_W - 1, LABELDST_NODE);
            ap_uint<LAB_W> labelsrc = edge.range(LABELSRC_NODE + LAB_W - 1, LABELSRC_NODE);
            ap_uint<NODE_W> nodedst = edge.range(DST_NODE + NODE_W - 1, DST_NODE);
            ap_uint<NODE_W> nodesrc = edge.range(SRC_NODE + NODE_W - 1, SRC_NODE);

            // Retrieve index of table with source as indexing vertex
            ap_uint<8> index0 = labelToTable[labelsrc][labeldst];
            // Retrieve index of table with destination as indexing vertex
            ap_uint<8> index1 = labelToTable[labeldst][labelsrc];

            /* Compute indices for hash table */
            ap_uint<LKP3_HASH_W> hash_out0;
            ap_uint<LKP3_HASH_W> hash_out1;

            xf::database::details::hashlookup3_core<NODE_W>(nodesrc, hash_out0);
            ap_uint<MAX_HASH_W> hashsrc = hash_out0.range(MAX_HASH_W - 1, 0);
            hashsrc = hashsrc.range(hash1_w - 1, 0);

            xf::database::details::hashlookup3_core<NODE_W>(nodedst, hash_out1);
            ap_uint<MAX_HASH_W> hashdst = hash_out1.range(MAX_HASH_W - 1, 0);
            hashdst = hashdst.range(hash1_w - 1, 0);

            /* Compute inside which block the edge will finish, in the meanwhile
            also adjust the edge as indexing -> indexed, and also add the hash
            values */
            unsigned short address_intable0 = hashsrc.range(hash1_w - 1, hash1_w - block_per_table);
            unsigned int address0 = ((index0 - 1) << block_per_table) + address_intable0;
            unsigned short address_intable1 = hashdst.range(hash1_w - 1, hash1_w - block_per_table);
            unsigned int address1 = ((index1 - 1) << block_per_table) + address_intable1;

            processed_edge_t table_edge0;
            table_edge0.range(31, 0) = nodesrc;
            table_edge0.range(63, 32) = nodedst;
            table_edge0.range(95, 64) = hashsrc;
            table_edge0.range(127, 96) = hashdst.range(hash2_w - 1, 0);

            processed_edge_t table_edge1;
            table_edge1.range(31, 0) = nodedst;
            table_edge1.range(63, 32) = nodesrc;
            table_edge1.range(95, 64) = hashdst;
            table_edge1.range(127, 96) = hashsrc.range(hash2_w - 1, 0);

            /* This useless if is to explain to Vitis HLS 2022.2 that two write in
            * the same stream cannot happen in one cycle */
            if (index0 != 0 && index1 != 0) {
              if (inverted) {
                    stream_edge[1].write({ address0, table_edge0, false });
                    stream_edge[0].write({ address1, table_edge1, false });
              } else {
                    stream_edge[0].write({ address0, table_edge0, false });
                    stream_edge[1].write({ address1, table_edge1, false });
              }
            } else if (index0 != 0) {
              if (inverted) {
                    stream_edge[1].write({ address0, table_edge0, false });
              } else {
                    stream_edge[0].write({ address0, table_edge0, false });
              }
            } else if (index1 != 0) {
              if (inverted) {
                    stream_edge[1].write({ address1, table_edge1, false });
              } else {
                    stream_edge[0].write({ address1, table_edge1, false });
              }
            }

            if ((index0 != 0) ^ (index1 != 0)) {
              inverted = !inverted;
            }
          }
        }
    }

    // if (inverted) {
    //     stream_edge[1].write({ 0, 0, true });
    // } else {
    //     stream_edge[0].write({ 0, 0, true });
    // }

  // Send a stop signal to BOTH consumer streams to prevent deadlock
  hls::print("[readAndStreamEdgesPerBlock] FINISHED: Sending STOP signal to stream[0]\n");
  stream_edge[0].write({ 0, 0, true });
  hls::print("[readAndStreamEdgesPerBlock] FINISHED: Sending STOP signal to stream[1]\n");
  stream_edge[1].write({ 0, 0, true });
}

template<size_t NODE_W,
         size_t LAB_W,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t MAX_LABELS>
void
storeEdgesPerBlock(hls::stream<store_tuple_t<processed_edge_t> > stream_edge[2],
                  row_t* m_axi, /* Target memory (scratchpad buffer bloom_p) */
                  unsigned int block_n_edges[4096])
{
    const size_t BRAM_LAT = 3;
    unsigned int local_cache_address[BRAM_LAT];
    unsigned int local_cache_counter[BRAM_LAT];
    ap_uint<BRAM_LAT> local_cache_valid = 0;
    int stopped_streams = 0;

#if DEBUG_PRINTS
    hls::print("\n[storeEdgesPerBlock] STARTING.\n", 0);
#endif

STORE_EDGES_PER_BLOCK_LOOP:
    while (stopped_streams < 2) {
#pragma HLS dependence variable = block_n_edges type = inter direction = RAW false
#pragma HLS dependence variable = m_axi type = inter direction = RAW false
#pragma HLS pipeline II = 1

        for (int i = 0; i < 2; i++) {
#pragma HLS unroll
            store_tuple_t<processed_edge_t> tuple_in;
            if (stream_edge[i].read_nb(tuple_in)) {
#if DEBUG_PRINTS
                hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Read from stream[%d]\n", i);
                hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Read successful, received first tuple. Address=%d\n", (unsigned int)tuple_in.address);
                hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Read successful, received first tuple. Edge=(%d, ", (unsigned int)tuple_in.edge.range(63, 32));
                hls::print("%d)\n", (unsigned int)tuple_in.edge.range(31, 0));
                hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Read successful, received first tuple. Stop=%d\n", (unsigned int)tuple_in.stop);
#endif
                if (tuple_in.stop) {
                    stopped_streams++;
#if DEBUG_PRINTS
                    hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: STOP signal received from stream[%d]\n", i);
                    hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Total stopped: %d\n", stopped_streams);
#endif
                } else {
                    unsigned int address = tuple_in.address;
#if DEBUG_PRINTS
                    hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Processing address %d\n", (unsigned int)address);
#endif
                    bool hit = false;
                    unsigned int local_value_counter = 0;

                    /* Check if the counter has been used recently, by cycling backword to
                    * catch the updated value */
                    for (auto s = 0; s < BRAM_LAT; s++) {
#pragma HLS unroll
                      auto g = BRAM_LAT - s - 1;
                      if (local_cache_address[g] == address && local_cache_valid[g]) {
                          hit = true;
                          local_value_counter = local_cache_counter[g];
                      }
                    }

                    /* Read from memory only if is not present in local cache, in this way
                    * it is possible to remove the RAW dependency */
                    if (!hit){
                      local_value_counter = block_n_edges[address];
                    }

                    /* Calculate the 512-bit word address and the 128-bit slot index */
                    unsigned int word_addr = local_value_counter / INSTR_PER_WORD;
                    unsigned int slot_idx = local_value_counter % INSTR_PER_WORD;
#if DEBUG_PRINTS
                    hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Block=%d\n", (unsigned int)address);
                    hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Word Address=%d\n", (unsigned int)word_addr);
                    hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Slot Index=%d\n", (unsigned int)slot_idx);
                    hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Writing edge (%d, ", (unsigned int)tuple_in.edge.range(63, 32)); /*indexing_node*/
                    hls::print("%d) to output stream.\n", (unsigned int)tuple_in.edge.range(31, 0)); /* indexed_node */
                    hls::print("[STORE_EDGES_PER_BLOCK_LOOP]: Writing edge to scratchpad_buf[%d]\n", (unsigned int)local_value_counter);
#endif
                    /* Perform Read-Modify-Write */
                    row_t temp_word = m_axi[word_addr];
                    temp_word.range(INSTR_WIDTH * (slot_idx + 1) - 1, INSTR_WIDTH * slot_idx) = tuple_in.edge;
                    m_axi[word_addr] = temp_word;

                    /* Shift everything by one position and writes the last one in memory */
                    for (auto s = 0; s < BRAM_LAT - 1; s++) {
#pragma HLS unroll
                      auto g = BRAM_LAT - s - 1;
                      local_cache_address[g] = local_cache_address[g - 1];
                      local_cache_counter[g] = local_cache_counter[g - 1];
                      local_cache_valid[g] = local_cache_valid[g - 1];
                    }

                    local_cache_address[0] = address;
                    local_cache_counter[0] = local_value_counter + 1;
                    local_cache_valid[0] = true;
                    block_n_edges[address] = local_value_counter + 1;
#ifndef __SYNTHESIS__
                    assert(local_value_counter < UINT32_MAX);
#endif
                  }
            }
        }
    }
#if DEBUG_PRINTS
        hls::print("[storeEdgesPerBlock]: FINISHED.\n", 0);
#endif
    }

template<size_t NODE_W,
         size_t LAB_W,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t MAX_LABELS>
void
storeEdgePerBlockWrap(row_t* edge_buf,
                      row_t* block_buf,
                      const unsigned char hash1_w,
                      const unsigned char hash2_w,
                      const ap_uint<8> labelToTable[MAX_LABELS][MAX_LABELS],
                      const unsigned int numDataEdges,
                      unsigned int block_n_edges[4096])
{
#pragma HLS dataflow

    // Stream for raw edges with hashes
    hls::stream<store_tuple_t<processed_edge_t>, 30> stream_edge[2];

    // Intermediate stream for sorted, ready-to-pack edges
    hls::stream<processed_edge_t, 32> sorted_edge_stream;

    hls::stream<bool, 4> stop_stream;

    readAndStreamEdgesPerBlock<NODE_W,
                               LAB_W,
                               LKP3_HASH_W,
                               MAX_HASH_W,
                               MAX_LABELS>(edge_buf,
                                           hash1_w,
                                           hash2_w,
                                           labelToTable,
                                           numDataEdges,
                                           stream_edge);

    /* Reads from stream, sorts edges according to block for blockToHTB and writes packed data to DDR */
    storeEdgesPerBlock<NODE_W, LAB_W, LKP3_HASH_W, MAX_HASH_W, MAX_LABELS>(
      stream_edge, block_buf, block_n_edges);
}

template<size_t NODE_W,
         size_t ROW_LOG,
         size_t EDGE_LOG,
         size_t LAB_W,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t MAX_LABELS>
void blockToHTB(row_t* edge_buf,
           row_t* htb_buf,
           AdjHT* hTables,
           const unsigned char hash1_w,
           const unsigned char hash2_w,
           const unsigned int numTables,
           const unsigned int block_n_edges[4096])
{
    constexpr size_t COUNTERS_PER_BLOCK = 14;
    constexpr size_t IXG_NODE = 0;
    constexpr size_t IXD_NODE = 32;
    constexpr size_t IXG_HASH = 64;
    constexpr size_t IXD_HASH = 96;
    const unsigned int block_per_table = (1UL << (hash1_w + hash2_w - COUNTERS_PER_BLOCK));
    ap_uint<64> block_counter0[4096];
    ap_uint<64> block_counter1[4096];
#pragma HLS bind_storage variable=block_counter0 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=block_counter1 type=RAM_2P impl=URAM

#if DEBUG_PRINTS
    hls::print("\n[blockToHTB]: STARTING.\n", 0);
#endif

/* Loop 2^(COUNTERS_PER_BLOCK - 2) since one block is split in two memories and
 * every memory keep two counters per line*/
INITIALIZE_URAM_LOOP:
    for (auto g = 0; g < (1UL << (COUNTERS_PER_BLOCK - 2)); g++) {
#pragma HLS pipeline II = 1
        block_counter0[g] = 0;
        block_counter1[g] = 0;
    }

    unsigned int previous_edge_count = 0;   /* Tracks cumulative edges */
    auto prev_ntb = 0;
    unsigned int base_address = 0;

BLOCK_HTB_TOP_LOOP:
    for (auto s = 0; s < block_per_table * numTables; s++) {
        auto block_edges = block_n_edges[s] - previous_edge_count;

        unsigned long num_block_words = 0;
        unsigned int start_word_addr = previous_edge_count / INSTR_PER_WORD;  /* Tracks word index into scratchpad buffer */
        if (block_edges > 0) {
            unsigned int end_word_addr = (previous_edge_count + block_edges - 1) / INSTR_PER_WORD;
            num_block_words = end_word_addr - start_word_addr + 1;
        }

        auto ntb = s >> (hash1_w + hash2_w - COUNTERS_PER_BLOCK);
        if (prev_ntb != ntb){
            base_address = 0;
        }
#if DEBUG_PRINTS
        hls::print("[BLOCK_HTB_TOP_LOOP]: Processing block s=%d\n", (unsigned int)s);
        hls::print("[BLOCK_HTB_TOP_LOOP]: Processing block for Table ntb=%d\n", (unsigned int)ntb);
        hls::print("[BLOCK_HTB_TOP_LOOP]: Total block edges=%d\n", (unsigned int)block_edges);
        hls::print("[BLOCK_HTB_TOP_LOOP]: num_block_words=%d\n", (unsigned int)num_block_words);
        hls::print("[BLOCK_HTB_TOP_LOOP]: PASS 1. Counting edges into on-chip URAMs.\n", 0);
#endif

COUNT_EDGES_INSIDE_BLOCK_LOOP:
        for (auto g_word = 0; g_word < num_block_words; g_word++) {
#pragma HLS pipeline II = INSTR_PER_WORD

          row_t packed_edge = edge_buf[start_word_addr + g_word];
#if DEBUG_PRINTS
          ap_uint<NODE_W> ixg_node_val = packed_edge.range(IXG_NODE + NODE_W - 1, IXG_NODE);
          hls::print("[COUNT_EDGES_INSIDE_BLOCK_LOOP]: Reading packed edge word %d\n", (unsigned int)g_word);
          hls::print("[COUNT_EDGES_INSIDE_BLOCK_LOOP]: Reading word from scratchpad_buf[%d]\n", (unsigned int)(g_word + start_word_addr));
#endif
          for (int g_unpack = 0; g_unpack < INSTR_PER_WORD; g_unpack++) {
#pragma HLS unroll
            /* Process only relevant slots */
            unsigned int absolute_edge_index = (start_word_addr + g_word) * INSTR_PER_WORD + g_unpack;
            if (absolute_edge_index >= previous_edge_count && absolute_edge_index < block_n_edges[s]) {
              ap_uint<INSTR_WIDTH> edge = packed_edge.range(INSTR_WIDTH * (g_unpack+1) - 1, INSTR_WIDTH * g_unpack);
              ap_uint<NODE_W> indexing_hash = edge.range(IXG_HASH + NODE_W - 1, IXG_HASH);
              ap_uint<NODE_W> indexed_hash =  edge.range(IXD_HASH + NODE_W - 1, IXD_HASH);
#if DEBUG_PRINTS
          hls::print("[COUNT_EDGES_INSIDE_BLOCK_LOOP]: Unpacking instruction %d\n", (unsigned int)g_unpack);
          hls::print("[COUNT_EDGES_INSIDE_BLOCK_LOOP]: ixg_node=%d\n", (unsigned int)edge.range(IXG_NODE + NODE_W - 1, IXG_NODE));
          hls::print("[COUNT_EDGES_INSIDE_BLOCK_LOOP]: ixd_node=%d\n", (unsigned int)edge.range(IXD_NODE + NODE_W - 1, IXD_NODE));
#endif

              /* Computing the bucket in which the edge will be stored, 
              restricted to the block of counter in memory */
              ap_uint<COUNTERS_PER_BLOCK> address = indexing_hash;
              address <<= hash2_w;
              address += indexed_hash;

              ap_uint<64> row_counter0;
              ap_uint<64> row_counter1;
              ap_uint<64> row_counter;

              /* The first bit select which counter in the 64-bit word, the second
              * one select on which memory read. In this way counters are
              * consecutive inside a word */
              row_counter1 = block_counter1[(address >> 2)];
              row_counter0 = block_counter0[(address >> 2)];

              if (address.test(1)){
                  row_counter = row_counter1;
              } else {
                  row_counter = row_counter0;
              }

              if (address.test(0)){
                  ap_uint<32> counter = row_counter.range(63, 32);
                  row_counter.range(63, 32) = counter + 1;
              } else {
                  ap_uint<32> counter = row_counter.range(31, 0);
                  row_counter.range(31, 0) = counter + 1;
              }

              if (address.test(1)){
                  row_counter1 = row_counter;
              } else {
                  row_counter0 = row_counter;
              }

              block_counter1[(address >> 2)] = row_counter1;
              block_counter0[(address >> 2)] = row_counter0;
            }
          }
        }

COUNTERS_TO_OFFSETS_URAM_LOOP:
        for (auto g = 0; g < (1UL << (COUNTERS_PER_BLOCK - 2)); g++){
#pragma HLS pipeline II = 2
            ap_uint<64> counter0 = block_counter0[g];
            ap_uint<64> counter1 = block_counter1[g];
            ap_uint<64> offset0, offset1;
            offset0.range(31, 0) = base_address;
            offset0.range(63, 32) = base_address + counter0.range(31, 0);
            offset1.range(31, 0) = base_address + counter0.range(31, 0) + counter0.range(63, 32);
            offset1.range(63, 32) = base_address + counter0.range(31, 0) + counter0.range(63, 32) + counter1.range(31, 0);
            base_address += counter0.range(31, 0) + counter0.range(63, 32) + counter1.range(31, 0) + counter1.range(63, 32);
            block_counter0[g] = offset0;
            block_counter1[g] = offset1;
        }

#if DEBUG_PRINTS
        hls::print("[BLOCK_HTB_TOP_LOOP]: PASS 2. Scattering edges into final htb_buf locations.\n", 0);
#endif
STORE_EDGES_INSIDE_BLOCK_LOOP:
        for (auto g_word = 0; g_word < num_block_words; g_word++) {
#pragma HLS pipeline II = INSTR_PER_WORD
          row_t packed_edge = edge_buf[start_word_addr + g_word];
          for (int g_unpack = 0; g_unpack < INSTR_PER_WORD; g_unpack++) {
#pragma HLS unroll
            unsigned int absolute_edge_index = (start_word_addr + g_word) * INSTR_PER_WORD + g_unpack;
            if (absolute_edge_index >= previous_edge_count && absolute_edge_index < block_n_edges[s]) {
              ap_uint<INSTR_WIDTH> edge = packed_edge.range(INSTR_WIDTH * (g_unpack+1) - 1, INSTR_WIDTH * g_unpack);

              ap_uint<NODE_W> indexing_hash = edge.range(IXG_HASH + NODE_W - 1, IXG_HASH);
              ap_uint<NODE_W> indexed_hash = edge.range(IXD_HASH + NODE_W - 1, IXD_HASH);
              ap_uint<NODE_W> indexed_node = edge.range(IXD_NODE + NODE_W - 1, IXD_NODE);
              ap_uint<NODE_W> indexing_node = edge.range(IXG_NODE + NODE_W - 1, IXG_NODE);

              /* Computing the bucket in which the edge will be stored, 
              restricted to the block of offset in memory */
              ap_uint<COUNTERS_PER_BLOCK> address = indexing_hash;
              address <<= hash2_w;
              address += indexed_hash;

              ap_uint<64> row_offset0;
              ap_uint<64> row_offset1;
              ap_uint<64> row_offset;
              ap_uint<32>  offset;
              
              /* The first bit select which offset in the 64-bit word, the second
              * one select on which memory read. In this way OFFSETS are
              * consecutive inside a word */
              row_offset1 = block_counter1[(address >> 2)];
              row_offset0 = block_counter0[(address >> 2)];

              if (address.test(1)){
                  row_offset = row_offset1;
              } else {
                  row_offset = row_offset0;
              }

              if (address.test(0)){
                  offset = row_offset.range(63, 32);
                  row_offset.range(63, 32) = offset + 1;
              } else {
                  offset = row_offset.range(31, 0);
                  row_offset.range(31, 0) = offset + 1;
              }

              if (address.test(1)){
                  row_offset1 = row_offset;
              } else {
                  row_offset0 = row_offset;
              }

              block_counter1[(address >> 2)] = row_offset1;
              block_counter0[(address >> 2)] = row_offset0;

              const int EDGES_PER_512_WORD = DDR_WORD / 64;
              ap_uint<32> edge_64bit_index = (hTables[ntb].start_edges * EDGE_ROW) + offset;

              // Calculate the 512-bit word address and the 64-bit slot within it
              ap_uint<32> word_addr = edge_64bit_index / EDGES_PER_512_WORD;
              ap_uint<32> slot_index = edge_64bit_index % EDGES_PER_512_WORD;
#if DEBUG_PRINTS
              hls::print("[STORE_EDGES_INSIDE_BLOCK_LOOP]: Writing edge (%d, ", (unsigned int)indexing_node);
              hls::print("%d)\n", (unsigned int)indexed_node);
              hls::print("[STORE_EDGES_INSIDE_BLOCK_LOOP]: Writing edge to htb_buf[%d]\n", (unsigned int)edge_64bit_index);
              hls::print("[STORE_EDGES_INSIDE_BLOCK_LOOP]: Writing edge to word address=%d\n", (unsigned int)word_addr);
              hls::print("[STORE_EDGES_INSIDE_BLOCK_LOOP]: Writing edge to slot index=%d\n", (unsigned int)slot_index);
#endif
              // Perform the read-modify-write
              row_t temp_word = htb_buf[word_addr];
#if DEBUG_PRINTS              
              hls::print("[STORE_EDGES_INSIDE_BLOCK_LOOP]: Word BEFORE modify [511:256]=%s\n", temp_word.range(511, 256).to_string(16).c_str());
              hls::print("[STORE_EDGES_INSIDE_BLOCK_LOOP]: Word BEFORE modify [255:  0]=%s\n", temp_word.range(255, 0).to_string(16).c_str());
#endif
              temp_word.range(64 * (slot_index + 1) - 1, 64 * slot_index) = indexing_node.concat(indexed_node);
              htb_buf[word_addr] = temp_word;
#if DEBUG_PRINTS              
              hls::print("[STORE_EDGES_INSIDE_BLOCK_LOOP]: Word AFTER modify [511:256]=%s\n", temp_word.range(511, 256).to_string(16).c_str());
              hls::print("[STORE_EDGES_INSIDE_BLOCK_LOOP]: Word AFTER modify [255:  0]=%s\n", temp_word.range(255, 0).to_string(16).c_str());
#endif
            }
          }
        }

        /* Store the block counters, packing them in a row */
        const int COUNTER_WORDS_PER_512 = 4; // 4 * 128-bit words in a 512-bit word
        const int NUM_COUNTER_WORDS = (1UL << (COUNTERS_PER_BLOCK - 2));        
        row_t packed_counters;
#if DEBUG_PRINTS
        hls::print("[BLOCK_HTB_TOP_LOOP]: Storing offset tables into htb_buf.\n");
#endif

STORE_OFFSETS_BLOCK_LOOP:
        for (auto g = 0; g < NUM_COUNTER_WORDS; g++) {
#pragma HLS pipeline II=1
            int slot = g % COUNTER_WORDS_PER_512;

            // Pack two 64-bit counters into one 128-bit chunk
            ap_uint<128> counter_chunk;
            counter_chunk.range(63, 0)   = block_counter0[g];
            counter_chunk.range(127, 64) = block_counter1[g];

            // Place the 128-bit chunk into the 512-bit buffer
            packed_counters.range(128 * (slot + 1) - 1, 128 * slot) = counter_chunk;

            block_counter0[g] = 0;
            block_counter1[g] = 0;

            // Write to memory when the buffer is full or on the last element
            if (slot == (COUNTER_WORDS_PER_512 - 1) || g == (NUM_COUNTER_WORDS - 1)) {
                int word_addr = g / COUNTER_WORDS_PER_512;
#if DEBUG_PRINTS
                unsigned int dest_addr = word_addr + (s * (NUM_COUNTER_WORDS / COUNTER_WORDS_PER_512));
                if (packed_counters != 0) { // Only print if there's data
                  hls::print("[STORE_OFFSETS_BLOCK_LOOP]: Writing offsets to htb_buf[%d]\n", dest_addr);
                  hls::print("[STORE_OFFSETS_BLOCK_LOOP]: Content [511:256]: %s\n", packed_counters.range(511, 256).to_string(16).c_str());
                  hls::print("[STORE_OFFSETS_BLOCK_LOOP]: Content [255:  0]: %s\n", packed_counters.range(255, 0).to_string(16).c_str());
                }
#endif
                htb_buf[word_addr + (s * (NUM_COUNTER_WORDS / COUNTER_WORDS_PER_512))] = packed_counters;
            }
        }
        /* Update trackers for the next block */
        previous_edge_count = block_n_edges[s];
        prev_ntb = ntb;
    }
#if DEBUG_PRINTS
    hls::print("[blockToHTB] FINISHED.\n", 0);
#endif
}

template<typename T_CNT,
         size_t ROW_LOG,
         size_t EDGE_LOG,
         size_t NODE_W,
         size_t LAB_W,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t MAX_LABELS>
void
storeEdgesHTB(row_t* edge_buf,
              row_t* htb_buf,
              AdjHT* hTables,
              const unsigned char hash1_w,
              const unsigned char hash2_w,
              const unsigned int numTables,
              const unsigned int block_n_edges[4096])
{
    constexpr size_t OFFSETS_PER_BLOCK = 14;
    constexpr size_t IXG_NODE = 0;
    constexpr size_t IXD_NODE = 32;
    constexpr size_t IXG_HASH = 64;
    constexpr size_t IXD_HASH = 96;
    const unsigned int block_per_table = (1UL << (hash1_w + hash2_w - OFFSETS_PER_BLOCK));
    ap_uint<64> block_offset0[4096];
    ap_uint<64> block_offset1[4096];
#pragma HLS bind_storage variable=block_offset0 type=RAM_2P impl=URAM
#pragma HLS bind_storage variable=block_offset1 type=RAM_2P impl=URAM

    auto prev_offset = 0;
STORE_EDGES_TOP_LOOP:
    for (auto s = 0; s < block_per_table * numTables; s++) {
        auto block_edges = block_n_edges[s] - prev_offset;
        auto ntb = s >> (hash1_w + hash2_w - OFFSETS_PER_BLOCK);
        
        row_t row;
LOAD_URAM_OFFSETS_LOOP:
        for (auto g = 0; g < (1UL << (OFFSETS_PER_BLOCK - 2)); g++) {
#pragma HLS pipeline II = 1
            row = htb_buf[g + (s * (1UL << (OFFSETS_PER_BLOCK - 2)))];
            block_offset0[g] = row.range(63, 0);
            block_offset1[g] = row.range(127, 64);
        }

STORE_EDGES_BLOCK_LOOP:
        for (auto g = 0; g < block_edges; g++) {
#pragma HLS pipeline II = 2
            row_t edge = edge_buf[g + prev_offset];
            
            ap_uint<NODE_W> indexing_hash =
              edge.range(IXG_HASH + NODE_W - 1, IXG_HASH);
            ap_uint<NODE_W> indexed_hash =
              edge.range(IXD_HASH + NODE_W - 1, IXD_HASH);
            ap_uint<NODE_W> indexed_node =
              edge.range(IXD_NODE + NODE_W - 1, IXD_NODE);
            ap_uint<NODE_W> indexing_node =
              edge.range(IXG_NODE + NODE_W - 1, IXG_NODE);

            /* Computing the bucket in which the edge will be stored, 
            restricted to the block of offset in memory */
            ap_uint<OFFSETS_PER_BLOCK> address = indexing_hash;
            address <<= hash2_w;
            address += indexed_hash;
            ap_uint<64> row_offset0;
            ap_uint<64> row_offset1;
            ap_uint<64> row_offset;
            ap_uint<32>  offset;
            
            /* The first bit select which offset in the 64-bit word, the second
             * one select on which memory read. In this way OFFSETS are
             * consecutive inside a word */
            row_offset1 = block_offset1[(address >> 2)];
            row_offset0 = block_offset0[(address >> 2)];

            if (address.test(1)){
                row_offset = row_offset1;
            } else {
                row_offset = row_offset0;
            }

            if (address.test(0)){
                offset = row_offset.range(63, 32);
                row_offset.range(63, 32) = offset + 1;
            } else {
                offset = row_offset.range(31, 0);
                row_offset.range(31, 0) = offset + 1;
            }

            if (address.test(1)){
                row_offset1 = row_offset;
            } else {
                row_offset0 = row_offset;
            }

            block_offset1[(address >> 2)] = row_offset1;
            block_offset0[(address >> 2)] = row_offset0;
            
            /* Compute address of row that will store the edge */
            T_CNT addr_row_edge =
              hTables[ntb].start_edges + (offset >> (ROW_LOG - EDGE_LOG));

            /* Compute address of the edge inside the row */
            T_CNT addr_inrow = offset.range((ROW_LOG - EDGE_LOG) - 1, 0);

            /* Read, modify and write the edge */
            row_t row_edge = htb_buf[addr_row_edge];
            row_edge.range(((addr_inrow + 1) << EDGE_LOG) - 1,
                           addr_inrow << EDGE_LOG) =
              indexing_node.concat(indexed_node);

            /* Store offset and edge modified */
            htb_buf[addr_row_edge] = row_edge;
        }

        /* Store the block OFFSETS packing them in a row */
STORE_OFFSETS_BLOCK_LOOP:
        for (auto g = 0; g < (1UL << (OFFSETS_PER_BLOCK - 2)); g++) {
#pragma HLS pipeline II = 1
            row.range(63, 0) = block_offset0[g];
            row.range(127, 64) = block_offset1[g];
            htb_buf[g + (s * (1UL << (OFFSETS_PER_BLOCK - 2)))] = row;
        }
        prev_offset = block_n_edges[s];
    }
}

/* Function in charge of reading the starting vertices of partial solutions. 
 * While reading an indexing set, it is critical to transform it from a bag 
 * in which nodes are repeated in a set. */
template<size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t FULL_HASH_W,
         size_t NODE_W,
         size_t EDGE_LOG,
         size_t ROW_LOG,
         size_t MAX_CL>
void
mwj_batch(const unsigned char hash1_w,
          unsigned int &n_candidate,
          const unsigned int start_address,
          AdjHT* hTables,
          QueryVertex* qVertices,
          row_t* htb_buf)
{
  ap_uint<8> tableIndex = 0;
  ap_uint<32> minSize = (1UL << 32) - 1;
  ap_uint<32> minOff;
  ap_uint<NODE_W * 2> edge;
  ap_uint<NODE_W> vertex;
  ap_uint<NODE_W> set[MAX_CL];
  ap_uint<MAX_HASH_W> hash_buff, hash_new;
  unsigned char set_counter = 0;
  bool flag_buff = false;
  bool flag_new = true;
  hash_buff = hash_new = 0;
  // unsigned int rm_start = 0;

PROPOSE_TBINDEXING_LOOP:
  for (int g = 0; g < qVertices[0].numTablesIndexing; g++) {
    tableIndex = qVertices[0].tables_indexing[g];

    if (hTables[tableIndex].n_edges < minSize) {
      minSize = hTables[tableIndex].n_edges;
      minOff = hTables[tableIndex].start_edges;
    }
  }

  unsigned int rowstart = minOff;
  unsigned int rowend = minOff + (minSize >> (ROW_LOG - EDGE_LOG));
  unsigned int window_right =
    minSize.range((ROW_LOG - EDGE_LOG) - 1, 0) + ((rowend - rowstart) << (ROW_LOG - EDGE_LOG));
  unsigned int cnt = 0;
  unsigned int address = start_address;
  n_candidate = 0;

PROPOSE_READ_MIN_INDEXING_LOOP:
  for (unsigned int g = 0; g <= rowend - rowstart; g++) {
    row_t row = htb_buf[rowstart + g];
    for (unsigned int i = 0; i < EDGE_ROW; i++, cnt++) {
#pragma HLS unroll
      if (cnt < window_right) {
        edge = row.range((1UL << EDGE_LOG) - 1, 0);
        vertex = edge.range(NODE_W * 2 - 1, NODE_W);
        ap_uint<LKP3_HASH_W> hash_out;
        xf::database::details::hashlookup3_core<NODE_W>(vertex, hash_out);
        hash_new = hash_out.range(MAX_HASH_W - 1, 0);
        hash_new = hash_new.range(hash1_w - 1, 0);

        if (flag_buff && hash_buff == hash_new) {
          flag_new = true;
        EXTRACT_BAGTOSET_SETCHECKER_LOOP:
          for (int nSet = 0; nSet < set_counter; nSet++) {
            if (vertex == set[nSet]) {
              flag_new = false;
              break;
            }
          }
        } else {
          flag_new = true;
          set_counter = 0;
        }

#ifndef __SYNTHESIS__
        assert(set_counter < MAX_CL);
#endif
        if (flag_new) {
          set[set_counter++] = vertex;
#if DEBUG_STATS
          debug::start_set++;
#endif
          htb_buf[address++] = vertex;
          n_candidate++;
        }

        hash_buff = hash_new;
        flag_buff = true;
      }
      row >>= (1UL << EDGE_LOG);
    }
  }

#if DEBUG_STATS
    debug::batch_reads += ceil((rowend - rowstart) / 16.0);
    // std::cout << rm_start << " removed\n";
#endif
}


/* Reads two times the data graph and fills the data stuctures */
template<typename T_DDR,
         typename T_BLOOM,
         size_t EDGE_LOG,
         size_t CNT_LOG,
         size_t BLOOM_LOG,
         size_t K_FUN_LOG,
         size_t ROW_LOG,
         size_t NODE_W,
         size_t NODE_LOG,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t FULL_HASH_W,
         size_t LAB_W,
         size_t STREAM_D,
         size_t HTB_SPACE,
         size_t MAX_LABELS,
         size_t MAX_CL>
void
fillTablesURAM(row_t* edge_buf,
               T_DDR* htb_buf,
               T_DDR* htb_buf1,
               row_t* bloom_p,
               QueryVertex* qVertices,
               AdjHT* hTables0,
               AdjHT* hTables1,
               const ap_uint<8> labelToTable[MAX_LABELS][MAX_LABELS],
               const unsigned long dynfifo_space,
               unsigned int &n_candidate,
               unsigned int &start_candidate,
               const unsigned long numDataEdges,
               const unsigned short numTables,
               const unsigned char hash1_w,
               const unsigned char hash2_w)
{

    /* Resetting portion of memory dedicated to counters
     * 1 << HASH1_W * HASH2_W is the number of counters needed
     * for each table, then it should be divided by the number
     * of counters stored in each row which is 1 << (ROW_LOG - CNT_LOG)*/
    constexpr size_t COUNTERS_PER_BLOCK = 14;
    const unsigned long htb_size =
      (1UL << (hash1_w + hash2_w - (DDR_BIT - COUNTER_WIDTH)));
    const unsigned int block_per_table =
      (1UL << (hash1_w + hash2_w - COUNTERS_PER_BLOCK));
    unsigned long start_addr = 0;
    unsigned int block_n_edges[4096];
#pragma HLS bind_storage variable = block_n_edges type = RAM_T2P impl = BRAM

#ifndef __SYNTHESIS__
    unsigned long end_addr = numTables * htb_size;
#endif

#if DEBUG_PRINTS
    hls::print("\n[fillTablesURAM]: STARTING.\n", 0);
    hls::print("[fillTablesURAM]: Calculated htb_size (offset table words per graph table) = %u\n", (unsigned int)htb_size);
#endif

STORE_HASHTABLES_POINTER_LOOP:
    for (unsigned int ntb = 0; ntb < numTables; ntb++){
        hTables0[ntb].start_offset = start_addr;
#if DEBUG_PRINTS
        hls::print("[STORE_HASHTABLES_POINTER_LOOP]: Assigning to Table %d\n", ntb);
        hls::print("[STORE_HASHTABLES_POINTER_LOOP]: Offset start address = %u\n", (unsigned int)start_addr);
#endif
        start_addr += htb_size;
    }
    
INITIALIZE_BRAM_LOOP:
    for (auto g = 0; g < numTables * block_per_table; g++) {
#pragma HLS pipeline II = 1
        block_n_edges[g] = 0;
    }

    countEdgesPerBlockWrap<NODE_W, LAB_W, LKP3_HASH_W, MAX_HASH_W, MAX_LABELS>(
      &edge_buf[dynfifo_space],
      hash1_w,
      hash2_w,
      labelToTable,
      numDataEdges,
      block_n_edges);

    auto base_addr = 0;
COUNTER_TO_OFFSET_BLOCK_LOOP:
    for (auto g = 0; g < numTables * block_per_table; g++) {
#pragma HLS pipeline II = 1
        auto data = block_n_edges[g];
        block_n_edges[g] = base_addr;
        base_addr += data;
    }

    storeEdgePerBlockWrap<NODE_W, LAB_W, LKP3_HASH_W, MAX_HASH_W, MAX_LABELS>(
      &edge_buf[dynfifo_space],
      reinterpret_cast<row_t*>(bloom_p),
      hash1_w,
      hash2_w,
      labelToTable,
      numDataEdges,
      block_n_edges);

#if DEBUG_PRINTS
    hls::print("[fillTablesURAM]: Address after offset tables = %u\n", (unsigned int)start_addr);
#endif
    start_addr = (start_addr + (1UL << CACHE_WORDS_PER_LINE)) &
                 ~((1UL << CACHE_WORDS_PER_LINE) - 1);
#if DEBUG_PRINTS
    hls::print("[fillTablesURAM]: Address after alignment for edges = %u\n", (unsigned int)start_addr);
#endif

    unsigned int prev_offset = 0;
STORE_EDGES_POINTER_LOOP:
    for (unsigned short ntb = 0; ntb < numTables; ntb++) {
        hTables0[ntb].start_edges = start_addr;
        unsigned int offset = block_n_edges[((ntb + 1) * block_per_table) - 1];
        hTables0[ntb].n_edges = offset - prev_offset;
        prev_offset = offset;
#if DEBUG_PRINTS
        hls::print("[STORE_EDGES_POINTER_LOOP]: Assigning to Table %d\n", ntb);
        hls::print("[STORE_EDGES_POINTER_LOOP]: Assigning n_edges = %u\n", (unsigned int)hTables0[ntb].n_edges);
        hls::print("[STORE_EDGES_POINTER_LOOP]: Assigning start_edges = %u\n", (unsigned int)start_addr);
#endif
        start_addr += (hTables0[ntb].n_edges >> (ROW_LOG - EDGE_LOG)) + 1;
#if DEBUG_PRINTS
        unsigned int words_for_edges = (hTables0[ntb].n_edges >> (ROW_LOG - EDGE_LOG)) + 1;
        hls::print("[STORE_EDGES_POINTER_LOOP]: Calculated words needed for edges = %u\n", words_for_edges);
        hls::print("[STORE_EDGES_POINTER_LOOP]: Next start_addr = %u\n", (unsigned int)start_addr);
#endif
        start_addr = (start_addr + (1UL << CACHE_WORDS_PER_LINE)) &
                     ~((1UL << CACHE_WORDS_PER_LINE) - 1);
#ifndef __SYNTHESIS__
        assert(start_addr < HTB_SPACE);
#endif
        hTables1[ntb].start_offset = hTables0[ntb].start_offset;
        hTables1[ntb].start_edges = hTables0[ntb].start_edges;
        hTables1[ntb].n_edges = hTables0[ntb].n_edges;
    }

    blockToHTB<NODE_W,
               ROW_LOG,
               EDGE_LOG,
               LAB_W,
               LKP3_HASH_W,
               MAX_HASH_W,
               MAX_LABELS>(
      reinterpret_cast<row_t*>(bloom_p), htb_buf, hTables0, hash1_w, hash2_w, numTables, block_n_edges);

    writeBloom<T_DDR,
               T_BLOOM,
               CNT_LOG,
               ROW_LOG,
               NODE_W,
               NODE_LOG,
               EDGE_LOG,
               MAX_CL,
               LKP3_HASH_W,
               MAX_HASH_W,
               FULL_HASH_W,
               BLOOM_LOG,
               K_FUN_LOG,
               STREAM_D>(bloom_p, htb_buf, htb_buf1, hTables0, qVertices, n_candidate, start_addr, numTables, hash1_w);

    // mwj_batch<LKP3_HASH_W,
    //           MAX_HASH_W,
    //           FULL_HASH_W,
    //           NODE_W,
    //           EDGE_LOG,
    //           ROW_LOG,
    //           MAX_CL>(
    //   hash1_w, n_candidate, start_addr, hTables0, qVertices, htb_buf);

#if DEBUG_PRINTS
    hls::print("[fillTablesURAM]: Final calculated start_candidate address = %u\n", (unsigned int)start_addr);
#endif
    start_candidate = start_addr;
#ifndef __SYNTHESIS__
    end_addr = start_addr * (1UL << (ROW_LOG - 3)) + ((numTables * ((1 << hash1_w) + 1)) << (BLOOM_LOG - 3));
    std::cout << "Occupied " << end_addr << " bytes, " << 
        end_addr / (float)(1UL << 20) << " MB. " << std::endl;
#endif

#if DEBUG_STATS
    debug::bloom_fullness /= numTables * (1UL << hash1_w) * (1UL << BLOOM_LOG) * (1UL << K_FUN_LOG);
#endif  /* DEBUG_STATS */
#if DEBUG_PRINTS
    hls::print("[fillTablesURAM] FINISHED.\n", 0);
#endif
}

template<typename T_DDR,
         typename T_BLOOM,
         size_t EDGE_LOG,
         size_t CNT_LOG,
         size_t BLOOM_LOG,
         size_t K_FUN_LOG,
         size_t ROW_LOG,
         size_t NODE_W,
         size_t NODE_LOG,
         size_t LKP3_HASH_W,
         size_t MAX_HASH_W,
         size_t FULL_HASH_W,
         size_t LAB_W,
         size_t STREAM_D,
         size_t HTB_SPACE,
         size_t MAX_QV,
         size_t MAX_TB,
         size_t MAX_CL>
void
preprocess(row_t* edge_buf,
           T_DDR* htb_buf0,
           T_DDR* htb_buf1,
           row_t* bloom_p,
           QueryVertex* qVertices,
           AdjHT* hTables0,
           AdjHT* hTables1,
           const unsigned long dynfifo_space,
           unsigned int &n_candidate,
           unsigned int &start_candidate,
           unsigned short numQueryVert,
           unsigned short numQueryEdges,
           unsigned long numDataEdges,
           const unsigned char hash1_w,
           const unsigned char hash2_w)
{
    constexpr size_t MAX_LABELS = (1UL << LAB_W);
    unsigned short numTables = 0;
    ap_uint<8> labelToTable[MAX_LABELS][MAX_LABELS];

INITIALIZE_LABELTOTABLE_LOOP:
    for (int g = 0; g < MAX_LABELS; g++) {
        for (int s = 0; s < MAX_LABELS; s++) {
#pragma HLS pipeline II = 2
            labelToTable[g][s] = 0;
        }
    }

    // Calculate the number of 512-bit words the data graph occupies
    const unsigned long num_data_graph_words = (numDataEdges + INSTR_PER_WORD - 1) / INSTR_PER_WORD;

    buildTableDescriptors<MAX_QV, MAX_TB, NODE_W, LAB_W, MAX_LABELS>(
      &edge_buf[dynfifo_space + num_data_graph_words],
      qVertices,
      labelToTable,
      numTables,
      numQueryVert,
      numQueryEdges);

    fillTablesURAM<T_DDR,
                   T_BLOOM,
                   EDGE_LOG,
                   CNT_LOG,
                   BLOOM_LOG,
                   K_FUN_LOG,
                   ROW_LOG,
                   NODE_W,
                   NODE_LOG,
                   LKP3_HASH_W,
                   MAX_HASH_W,
                   FULL_HASH_W,
                   LAB_W,
                   STREAM_D,
                   HTB_SPACE,
                   MAX_LABELS,
                   MAX_CL>(edge_buf,
                               htb_buf0,
                               htb_buf1,
                               bloom_p,
                               qVertices,
                               hTables0,
                               hTables1,
                               labelToTable,
                               dynfifo_space,
                               n_candidate,
                               start_candidate,
                               numDataEdges,
                               numTables,
                               hash1_w,
                               hash2_w);

#if DEBUG_PRINTS
    hls::print("\n--- FINAL PREPROCESSING RESULTS ---\n", 0);
    hls::print("Final n_candidate = %d\n", n_candidate);
    hls::print("Candidate list starts at htb_buf[%d]\n\n", start_candidate);

    hls::print("--- hTables Contents ---\n", 0);
    for (int i = 0; i < numTables; ++i) {
      hls::print("Table %d\n", i);
      hls::print("start_offset=%u\n", (unsigned int)hTables0[i].start_offset);
      hls::print("start_edges=%u\n", (unsigned int)hTables0[i].start_edges);
      hls::print("n_edges=%u\n", (unsigned int)hTables0[i].n_edges);
    }

    hls::print("\n--- qVertices Contents ---\n");
    for (int i = 0; i < numQueryVert; ++i) {
      hls::print("qVertex %d\n", i);
      hls::print("numTablesIndexed=%d\n", (int)qVertices[i].numTablesIndexed);
      hls::print("numTablesIndexing=%d\n", (int)qVertices[i].numTablesIndexing);
      for (int j = 0; j < qVertices[i].numTablesIndexed; ++j) {
        hls::print("Indexed by v%d\n", (int)qVertices[i].vertex_indexing[j]);
        hls::print("In Table %d\n", (int)qVertices[i].tables_indexed[j]);
      }
    }
    hls::print("--- END OF PREPROCESSING ---\n\n", 0);
#endif
}

#pragma GCC diagnostic pop
