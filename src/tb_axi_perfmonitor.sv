////////////////////////////
// AXI4 Performance Monitor
//
//      ================================================================================
//      Overview
//      --------------------------------------------------------------------------------
//      This AXI4 Performance Monitor observes AXI transactions on its AXI4 port, records
//      statistical data about the transfers in per-burst granularity, and writes them
//      to a file.
//
//
//      ================================================================================
//      Read Statistics Legend
//      --------------------------------------------------------------------------------
//      Cycle:               | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |14 |15 |
//      CLK      ----->  /‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/
//      ARADDR   ----->  #[   A   ]########################################################
//      ARVALID  ----->  _/‾‾‾‾‾‾‾\________________________________________________________
//      ARREADY  ----->  _____/‾‾‾\########################################################
//      RDATA    ----->  #############################[  D0  ]####[  D1  ]####[  D2  ]#####
//      RLAST    ----->  #############################________________________/‾‾‾‾‾‾‾#####
//      RVALID   ----->  _____________________________/‾‾‾‾‾‾‾\___/‾‾‾‾‾‾‾\___/‾‾‾‾‾‾‾\____
//      RREADY   ----->  #############################____/‾‾‾####____/‾‾‾####____/‾‾‾\____
//      count as:            |RAS|RAC|LAG|LAG|LAG|LAG|LAG|RS |DAT|SRL|RS |DAT|SRL|RS |DAT|IDL|
//      Read Latency:        |<----------------------------->|   |<----->|   |<----->|
//                            ARVALID to RVALID && RREADY && RLAST minus burst length (12 cycles here)
//      Initiation Delay:    |<------------------------->|
//                            ARVALID to first RVALID (7 cycles here)
//      Throughput cycles:                               |<----------------------------->|
//                            RVALID && RREADY to RVALID && RREADY && RLAST (8 cycles here)
//      Transfer duration:   |<--------------------------------------------------------->|
//                            ARVALID to RVALID && RREADY && RLAST (15 cycles here)
//
//      Read cycle categorization:
//        -> https://zipcpu.com/img/axiperf/rdcategories.svg
//        -> https://zipcpu.com/img/axiperf/rdburst-annotated.svg
//
//      ================================================================================
//      Write Statistics Legend
//      --------------------------------------------------------------------------------
//                                      Address precedes/simultaneous to Data                     ~   Data precedes Address
//      Cycle:              | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |14 |15 |16 | ~ |17 |18 |19 |20 |21 |22 |
//      CLK      -----> /‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/ ~ /‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾
//      AWADDR   -----> #[   A   ]############################################################### ~ #############[ A ]########
//      AWVALID  -----> _/‾‾‾‾‾‾‾\_______________________________________________________________ ~ _____________/‾‾‾\________
//      AWREADY  -----> _____/‾‾‾\_______________________________________________________________ ~ _____________/‾‾‾\________
//      WDATA    -----> #####################[  D0  ][  D1  ]####[D2][D3]######################## ~ #[  D0   ]################
//      WLAST    -----> #####################________________________/‾‾‾\####################### ~ #____/‾‾‾\________________
//      WVALID   -----> _____________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\___/‾‾‾‾‾‾‾\_______________________ ~ _/‾‾‾‾‾‾‾\________________
//      WREADY   -----> #________________________/‾‾‾\___/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\####################### ~ ____/‾‾‾‾\________________
//      BRESP    -----> #############################################################[OK ]####### ~ #################[OK ]###
//      BVALID   -----> _____________________________________________________/‾‾‾‾‾‾‾‾‾‾‾\_______ ~ _________________/‾‾‾\____
//      BREADY   -----> _____________________________________________________________/‾‾‾\_______ ~ _________________/‾‾‾\____
//      count as:           |AS |AW |LAG|LAG|LAG|WS1|DAT|WS1|DAT|SWD|DAT|DAT|BLG|BST|BST|EOB|IDL| ~ |IDL|EDS|EWB|LAG|WAL|EOB|
//      Write Latency:      |<--------------------->|   |<->|   |<->|       |<------------->|     ~     |<->|   |<--------->|
//                           AWVALID to BAVLID && BREADY minus burst length (12 cycles here)      ~ WVALID to BVALID && BREADY minus burst length
//      Initiation Delay:   |<----------------->|                                                 ~     |<-------------->|
//                           AWVALID to first WVALID (5 cycles here)                              ~ WVALID to first AWVALID
//      Throughput cycles:                      |<------------------------->|                     ~     |<----->|
//                           WVALID to BVALID && BREADY (7 cycles here)                           ~ first WVALID to WVALID && WREADY && WLAST
//      Transfer duration:  |<------------------------------------------------------------->|     ~     |<----------------->|
//                           AWVALID to BVALID && BREADY (16 cycles here)                         ~ WVALID to BVALID && BREADY
//
//      Write cycle categorization:
//        -> https://zipcpu.com/img/axiperf/wrcategories.svg
//        -> https://zipcpu.com/img/axiperf/wrburst-annotated.svg
//

`timescale 1ps/1ps

module tb_axi_perfmonitor(
	input wire clk,
	input wire nres,
    input wire stopclock,
    input wire manual_start,
    input wire manual_stop,
    input wire manual_clear,
    input wire manual_print,
	AXI_BUS   axi_bus
);


//------------------------------------------------------------------------------
// Parameters
//------------------------------------------------------------------------------
    // Parameters below can be set by the user.

    // Select the number of channel ID bits required
    parameter WID_WIDTH = 4;            // (AW|B)ID width
    parameter RID_WIDTH = 4;            // (AR|R)ID width

    // Size of secondary matrix dimension for storing outstanding read bursts, this should match or
    // exceed the number of outstanding read addresses accepted into the slave interface
    // interface
    parameter MAXRBURSTS = 16;

    // Size of secondary matrix dimension for storing outstanding write bursts, this should match or
    // exceed the number of outstanding write bursts into the slave interface
    parameter MAXWBURSTS = 16;

    // Set ADDR_WIDTH to the address-bus width required
    parameter ADDR_WIDTH = 32;

    // Name for output files
    parameter OUT_FILE_NAME = "perfmonitor_out";
    parameter RAW_OUT_FILE_NAME = {OUT_FILE_NAME, "_raw"};

    // Set to 1 to dump raw data to csv file
    parameter DUMP_RAW_DATA = 0;

    // switch to control if measurement shall be started/stopped with first transfer on bus
    // or by external start/stop signal
    parameter MANUAL_CONTROL = 1'b0;

    // size of read/write stats buffer
    parameter MAX_NUM_REC_BURSTS = 10000;

    // address filter
    parameter logic [ADDR_WIDTH-1:0] FILTER_ADDR = 'b0;
    parameter logic [ADDR_WIDTH-1:0] FILTER_MASK = {ADDR_WIDTH{1'b1}};
    parameter logic FILTER_EN = 1'b0;

    //Do not override local parameters below!
    localparam ID_MAX_R   = RID_WIDTH? RID_WIDTH-1:0;    // ID max index
    localparam ID_MAX_W   = WID_WIDTH? WID_WIDTH-1:0;    // ID max index
    localparam ID_MAX     = ID_MAX_W > ID_MAX_R ? ID_MAX_W : ID_MAX_R;    // ID max index

    localparam MAXRBURSTS_WIDTH = $clog2(MAXRBURSTS + 1'b1);
    localparam MAXWBURSTS_WIDTH = $clog2(MAXWBURSTS + 1'b1);

    localparam MAXCNT = 32;
    localparam TIMESTAMP_WIDTH = $bits(time);

//------------------------------------------------------------------------------
//  Variables
//------------------------------------------------------------------------------

// time counters
logic [MAXCNT-1:0] active_count, idle_count;
// module control signals
logic start_measurement, stop_measurement, clear_measurement, print_measurement,
        rd_rec_triggered, r_rd_rec_triggered, wr_rec_triggered, r_wr_rec_triggered, start_rcvd, perf_err,
        active_count_err, temp_stats_created; //idle_bus, r_idle_bus;

logic [(1<<RID_WIDTH)-1:0][MAXRBURSTS-1:0] rd_no_increment; // read burst ID and index for which no increment of the read stats matrix was done (for active bursts)
logic [(1<<WID_WIDTH)-1:0][MAXWBURSTS-1:0] wr_no_increment; // write burst ID and index for which no increment of the write stats matrix was done (for active bursts)
logic [MAXWBURSTS-1:0] temp_wr_no_increment;                // write burst index for which no increment of any counters in temp_stats_arr was done (for active bursts)
logic merge_wr_no_increment;                                // flag to indicate that no increment of any counters during merging of statistics from temp_stats_arr to write_stats_matrix was done

// total write counters
logic [MAXCNT-1:0] wr_total_awvalid_count, wr_total_beat_count, wr_total_burst_count,
                    wr_total_bvalid_count, wr_total_wvalid_count, wr_total_idle_count,
                    wr_total_bus_active, wr_total_addr_stall_count, wr_total_addr_lag_count,
                    wr_total_early_stall_count, wr_total_b_lag_count, wr_total_b_stall_count,
                    wr_total_b_end_count, wr_total_data_lag_count, wr_total_stall_count,
                    wr_total_slow_data_count, wr_total_awr_early_count, wr_total_early_beat_count,
                    wr_total_throughput_dnm_count;
logic [MAXCNT-1:0] r_wr_total_addr_stall_count, r_wr_total_addr_lag_count, r_wr_total_early_stall_count,
                    r_wr_total_b_lag_count, r_wr_total_b_stall_count, r_wr_total_data_lag_count,
                    r_wr_total_stall_count, r_wr_total_slow_data_count, r_wr_total_awr_early_count,
                    r_wr_total_early_beat_count;
// write control signals and burst spanning counters
logic wr_in_progress, wr_w_zero_outstanding, wr_aw_zero_outstanding, wr_awvalid_zero_unmatched,
    wr_wvalid_zero_unmatched, wr_aw_err, wr_w_err, wr_awvalid_err, wr_wvalid_err, wr_first_in_rec,
    wr_channel_idle, r_wr_channel_idle, wr_channel_idle_reg, wr_recording_saved, awvalid_occured,
    wvalid_occured;
logic [MAXWBURSTS_WIDTH-1:0] wr_aw_outstanding, wr_w_outstanding, wr_awvalid_unmatched, wr_wvalid_unmatched;
logic [MAXWBURSTS_WIDTH-1:0] wr_aw_max_outstanding, wr_w_max_outstanding;
logic [(1<<WID_WIDTH)-1:0] wr_recording_done;


// Flags to ensure only singular increments/decrements of awvalid/wvalid_unmatched counters per burst
// logic wr_lock_awvalid_increment, wr_lock_awvalid_decrement, wr_lock_wvalid_increment, wr_lock_wvalid_decrement;

// total read counters
logic [MAXCNT-1:0] rd_total_idle_count, rd_total_beat_count, rd_total_burst_count,
                rd_total_ar_stall_count, rd_total_r_stall_count, rd_total_arvalid_count,
                rd_total_rvalid_count, rd_total_bus_active_count, rd_total_slow_link_count,
                rd_total_ar_cycle_count, rd_total_lag_count, rd_total_lag_cycles,
                rd_total_throughput_dnm_count, rd_first_lag;
logic [MAXCNT-1:0] r_rd_total_r_stall_count, r_rd_total_slow_link_count, r_rd_total_lag_count,
                r_rd_total_ar_stall_count;
// read control signals and ID spanning counters
logic rd_err, rd_ar_err, rd_first_in_rec, rd_zero_outstanding, rd_ar_nonzero_outstanding, arvalid_occured,
        rd_channel_idle, r_rd_channel_idle, rd_channel_idle_reg, rd_recording_saved;
logic [MAXRBURSTS_WIDTH-1:0] rd_outstanding_bursts, rd_ar_outstanding, rd_total_in_flight;
logic [MAXRBURSTS_WIDTH-1:0] rd_max_outstanding;

logic [(1<<RID_WIDTH)-1:0] rd_recording_done, rd_bursts_in_flight;

// variables for managing appending, popping or updating matrix/temp_stats_arr elements
// logic [ID_MAX_W:0] wr_wlast_outstanding [$];  // queue of burst AWIDs for burst addresses that did not yet experience a WLAST for their corresponding data, as well as a temp variable
int first_unmatched_wlast_idx;  // temp variable to store the first index for a given ID for which the WLAST is still outstanding
logic merge_cycle;   // flag that indicates a merge for temp_stats into write_stats_matrix happening this cycle
logic [(1<<WID_WIDTH)-1:0] write_matrix_full, write_matrix_empty;
logic [(1<<RID_WIDTH)-1:0] read_matrix_full, read_matrix_empty;
logic temp_write_stats_full, temp_write_stats_empty;
int write_matrix_last[(1<<WID_WIDTH)], temp_write_stats_last;
int read_matrix_last[(1<<RID_WIDTH)];
int write_stats_last, read_stats_last;

//------------------------------------------------------------------------------
//  Data structures
//------------------------------------------------------------------------------
    // Define struct for read burst statistics
    typedef struct {
        //common data fields
        logic [ADDR_WIDTH-1:0]  address = 0;
        logic [TIMESTAMP_WIDTH-1:0]            time_issued = 0, time_completed = 0;
        logic [MAXCNT-1:0]      duration = 0, latency = 0, init_delay = 0,
                                throughput_dnm = 0; // throughput denominator
        logic [7:0]             burst_length = 0;
        //read transfer specific data fields
        logic [ID_MAX_R:0]      id = 0;
        //                      r_stall_count is read backpressure
        logic [MAXCNT-1:0]      r_stall_count = 0, slow_link_count = 0, lag_count = 0,
                                ar_stall_count = 0, ar_cycle_count = 0, beat_count = 0;
        logic   rd_ar_outstanding = 1'b0;
    } read_stats_t;

    // Define struct for write burst statistics
    typedef struct {
        //common data fields
        logic [ADDR_WIDTH-1:0]  address = 0;
        logic [TIMESTAMP_WIDTH-1:0]            time_issued = 0, time_completed = 0;
        logic [MAXCNT-1:0]      duration = 0, latency = 0, init_delay = 0,
                                throughput_dnm = 0, backpressure = 0; // throughput denominator
        logic [7:0]             burst_length = 0;
        //write transfer specific data fields
        logic [ID_MAX_W:0]      id = 0;
        // counter values at beginning of burst
        logic [MAXCNT-1:0]      stall_count = 0, beat_count = 0, b_lag_count = 0,
                                b_stall_count = 0, b_end_count = 0, early_stall_count = 0,
                                slow_data_count = 0, early_beat_count = 0,
                                addr_lag_count = 0, data_lag_count = 0, awr_early_count = 0,
                                addr_stall_count = 0;
        logic wr_aw_outstanding = 1'b0, wr_w_outstanding = 1'b0, wr_in_progress = 1'b0;
    } write_stats_t;

    // Define struct for temporary write burst statistics -> is to be merged into write_stats_t
    typedef struct {
        logic [TIMESTAMP_WIDTH-1:0]            time_issued = 0;
        // counter values at beginning of burst
        logic [MAXCNT-1:0]      stall_count = 0, beat_count = 0, early_stall_count = 0,
                                early_beat_count = 0, addr_lag_count = 0, addr_stall_count = 0;
        logic   wr_w_outstanding = 1'b0, wr_in_progress = 1'b0;
    } temp_write_stats_t;

    typedef read_stats_t read_stats_matrix_t[(1<<RID_WIDTH)][MAXRBURSTS];
    typedef write_stats_t write_stats_matrix_t[(1<<WID_WIDTH)][MAXWBURSTS];

    typedef struct {
        logic [ID_MAX_W:0]              id = 0;     // ID of the write burst in the write_stats_matrix
        logic [MAXWBURSTS_WIDTH-1:0]    idx = 0;    // index of the burst in the write_stats_matrix1
    } token_t;

    // instantiation of data structures
    read_stats_matrix_t read_stats_matrix;
    write_stats_matrix_t write_stats_matrix;
    temp_write_stats_t temp_write_stats_arr[MAXWBURSTS];
    read_stats_t read_stats_arr[MAX_NUM_REC_BURSTS];
    write_stats_t write_stats_arr[MAX_NUM_REC_BURSTS];

    token_t w_channel_token_queue[$];

//------------------------------------------------------------------------------
//  Function Declarations
//------------------------------------------------------------------------------
    // appends an entry to the output buffer of read stats
    function automatic void read_stats_append(
        ref read_stats_t read_stats_arr[MAX_NUM_REC_BURSTS],
        ref read_stats_t new_stats
    );
        //check if there is enough space in the array
        if (read_stats_last == MAX_NUM_REC_BURSTS-1) begin
            $error("read stats buffer overflow");
            return;
        end
        read_stats_arr[++read_stats_last] = new_stats;
        return;
    endfunction

    // appends an entry to the output buffer of write stats
    function automatic void write_stats_append(
        ref write_stats_t write_stats_arr[MAX_NUM_REC_BURSTS],
        ref write_stats_t new_stats
    );
        //check if there is enough space in the array
        if (write_stats_last == MAX_NUM_REC_BURSTS-1) begin
            $error("write stats buffer overflow");
            return;
        end
        write_stats_arr[++write_stats_last] = new_stats;
        return;
    endfunction

    // appends an entry to the read matrix which monitors currently active read transactions
    function automatic int read_matrix_append(
        ref read_stats_matrix_t read_matrix,
        ref read_stats_t new_transfer
    );
        // Check if there is enough space in the array
        if (read_matrix_full[new_transfer.id]) begin
            $error("read burst buffer overflow for ID:%d", new_transfer.id);
            return 0;
        end
        // Append the new transfer data to the array
        read_matrix[new_transfer.id][++(read_matrix_last[new_transfer.id])] = new_transfer;
        if (read_matrix_last[new_transfer.id] == MAXRBURSTS-1) begin
            read_matrix_full[new_transfer.id] = 1'b1;
        end
        read_matrix_empty[new_transfer.id] = 1'b0;
        return 1;
    endfunction

    // returns the top entry for a given id from the read matrix
    function automatic int read_matrix_pop(
        ref read_stats_t read_stats_buf,
        ref read_stats_matrix_t read_matrix,
        input int id
    );
        // Check if ID exists
        if (id > ((1<<RID_WIDTH) - 1)) begin
            $error("ID %d accessed in read_matrix doesn't exist", id);
            return 0;
        end
        // Check if any bursts for this ID exist
        if (read_matrix_empty[id]) begin
            $error("No bursts for ID %d in read_matrix", id);
            return 0;
        end

        read_stats_buf = read_matrix[id][0];
        for (int i=0; i<MAXRBURSTS; i++) begin
            if (i < read_matrix_last[id]) begin
                // read_matrix[id][i] = read_matrix[id][i+1];
                // Shift the entries in the matrix except time_completed, since this would overwritten if the recording at index 1 finishes in the same cycle as the shift
                read_matrix[id][i].address              = read_matrix[id][i+1].address;
                read_matrix[id][i].time_issued          = read_matrix[id][i+1].time_issued;
                read_matrix[id][i].duration             = read_matrix[id][i+1].duration;
                read_matrix[id][i].latency              = read_matrix[id][i+1].latency;
                read_matrix[id][i].init_delay           = read_matrix[id][i+1].init_delay;
                read_matrix[id][i].throughput_dnm       = read_matrix[id][i+1].throughput_dnm;
                read_matrix[id][i].burst_length         = read_matrix[id][i+1].burst_length;
                read_matrix[id][i].id                   = read_matrix[id][i+1].id;
                read_matrix[id][i].r_stall_count        = read_matrix[id][i+1].r_stall_count;
                read_matrix[id][i].slow_link_count      = read_matrix[id][i+1].slow_link_count;
                read_matrix[id][i].lag_count            = read_matrix[id][i+1].lag_count;
                read_matrix[id][i].ar_stall_count       = read_matrix[id][i+1].ar_stall_count;
                read_matrix[id][i].ar_cycle_count       = read_matrix[id][i+1].ar_cycle_count;
                read_matrix[id][i].beat_count           = read_matrix[id][i+1].beat_count;
                read_matrix[id][i].rd_ar_outstanding    = read_matrix[id][i+1].rd_ar_outstanding;
            end else begin
                read_matrix[id][i] = '{default: 'b0};
            end
        end
        if (read_matrix_full[id]) begin
            read_matrix_full[id] = 1'b0;
        end
        if ((read_matrix_last[id]--) == 0) begin
            read_matrix_empty[id] = 1'b1;
        end
        return 1;
    endfunction

    // appends an entry to the write matrix which monitors currently active write transactions (for which both address and data handshake have happened/are currently happening)
    function automatic int write_matrix_append(
        ref write_stats_matrix_t write_matrix,
        ref write_stats_t new_transfer
    );
        // Check if there is enough space in the array
        if (write_matrix_full[new_transfer.id]) begin
            $error("write burst buffer overflow for ID:%d", new_transfer.id);
            return 0;
        end
        // Append the new transfer data to the array
        write_matrix[new_transfer.id][++(write_matrix_last[new_transfer.id])] = new_transfer;
        if (write_matrix_last[new_transfer.id] == MAXWBURSTS-1) begin
            write_matrix_full[new_transfer.id] = 1'b1;
        end
        write_matrix_empty[new_transfer.id] = 1'b0;
        return 1;
    endfunction

    function automatic int write_matrix_pop(
        ref write_stats_t write_stats_buf,
        ref write_stats_matrix_t write_matrix,
        input int id
    );
        // Check if ID exists
        if (id > ((1<<WID_WIDTH) - 1)) begin
            $error("ID %d accessed in write_bursts_matrix doesn't exist", id);
            return 0;
        end
        // Check if any bursts for this ID exist
        if (write_matrix_empty[id]) begin
            $error("No bursts for ID %d in write_bursts_matrix", id);
            return 0;
        end

        write_stats_buf = write_matrix[id][0];
        for (int i=0; i<MAXWBURSTS; i++) begin
            if (i < write_matrix_last[id]) begin
                // write_matrix[id][i] = write_matrix[id][i+1];
                // Shift the entries in the matrix except time_completed, since this would overwritten if the recording at index 1 finishes in the same cycle as the shift
                write_matrix[id][i].address             = write_matrix[id][i+1].address;
                write_matrix[id][i].time_issued         = write_matrix[id][i+1].time_issued;
                write_matrix[id][i].duration            = write_matrix[id][i+1].duration;
                write_matrix[id][i].latency             = write_matrix[id][i+1].latency;
                write_matrix[id][i].init_delay          = write_matrix[id][i+1].init_delay;
                write_matrix[id][i].throughput_dnm      = write_matrix[id][i+1].throughput_dnm;
                write_matrix[id][i].backpressure        = write_matrix[id][i+1].backpressure;
                write_matrix[id][i].burst_length        = write_matrix[id][i+1].burst_length;
                write_matrix[id][i].id                  = write_matrix[id][i+1].id;
                write_matrix[id][i].stall_count         = write_matrix[id][i+1].stall_count;
                write_matrix[id][i].beat_count          = write_matrix[id][i+1].beat_count;
                write_matrix[id][i].b_lag_count         = write_matrix[id][i+1].b_lag_count;
                write_matrix[id][i].b_stall_count       = write_matrix[id][i+1].b_stall_count;
                write_matrix[id][i].b_end_count         = write_matrix[id][i+1].b_end_count;
                write_matrix[id][i].early_stall_count   = write_matrix[id][i+1].early_stall_count;
                write_matrix[id][i].slow_data_count     = write_matrix[id][i+1].slow_data_count;
                write_matrix[id][i].early_beat_count    = write_matrix[id][i+1].early_beat_count;
                write_matrix[id][i].addr_lag_count      = write_matrix[id][i+1].addr_lag_count;
                write_matrix[id][i].data_lag_count      = write_matrix[id][i+1].data_lag_count;
                write_matrix[id][i].awr_early_count     = write_matrix[id][i+1].awr_early_count;
                write_matrix[id][i].addr_stall_count    = write_matrix[id][i+1].addr_stall_count;
                write_matrix[id][i].wr_aw_outstanding   = write_matrix[id][i+1].wr_aw_outstanding;
                write_matrix[id][i].wr_w_outstanding    = write_matrix[id][i+1].wr_w_outstanding;
                write_matrix[id][i].wr_in_progress      = write_matrix[id][i+1].wr_in_progress;
            end else begin
                write_matrix[id][i] = '{default: 'b0};
            end
        end
        if (write_matrix_full[id]) begin
            write_matrix_full[id] = 1'b0;
        end
        if ((write_matrix_last[id]--) == 0) begin
            write_matrix_empty[id] = 1'b1;
        end

        // decrement all indices of the specified ID in the token queue
        for (int i=0; i < w_channel_token_queue.size(); i++) begin
            if (w_channel_token_queue[i].id == id) begin
                w_channel_token_queue[i].idx--;
            end
        end
        return 1;
    endfunction

    function automatic int temp_write_stats_append(
        ref temp_write_stats_t temp_write_stats_arr[MAXWBURSTS],
        ref temp_write_stats_t new_transfer
    );
        // Check if there is enough space in the array
        if (temp_write_stats_full) begin
            $error("write burst buffer overflow for temp write stats");
            return 0;
        end
        // Append the new transfer data to the dynamic array
        temp_write_stats_arr[++(temp_write_stats_last)] = new_transfer;
        if (temp_write_stats_last == MAXWBURSTS-1) begin
            temp_write_stats_full = 1'b1;
        end
        temp_write_stats_empty = 1'b0;
        return 1;
    endfunction

    function automatic int temp_write_stats_pop(
        ref temp_write_stats_t temp_write_stats_buf,
        ref temp_write_stats_t temp_write_stats_arr[MAXWBURSTS]
    );
        // Check if any bursts exist
        if (temp_write_stats_empty) begin
            $error("No bursts in temp_write_stats_arr");
            return 0;
        end

        temp_write_stats_buf = temp_write_stats_arr[0];
        for (int i=0; i<MAXWBURSTS; i++) begin
            if (i < temp_write_stats_last) begin
                temp_write_stats_arr[i] = temp_write_stats_arr[i+1];
            end else begin
                temp_write_stats_arr[i] = '{default: 'b0};
            end
        end
        if (temp_write_stats_full) begin
            temp_write_stats_full = 1'b0;
        end
        if ((temp_write_stats_last--) == 0) begin
            temp_write_stats_empty = 1'b1;
        end
        return 1;
    endfunction

    function automatic void read_burst_start_record2matrix(
        ref read_stats_matrix_t read_matrix
    );
        read_stats_t read_stats;
        read_stats.address      = axi_bus.ar_addr;
        read_stats.time_issued  = $time;
        read_stats.burst_length = axi_bus.ar_len;
        read_stats.id           = axi_bus.ar_id;
        // bin initial clock cycle, since normal counter operation starts only on the next cycle
        if (axi_bus.ar_ready) begin
            read_stats.ar_cycle_count       = MAXCNT'('d1);
            read_stats.rd_ar_outstanding    = 1'b1;
        end
        else begin
            read_stats.ar_stall_count   = MAXCNT'('d1);
        end
        if (!read_matrix_append(read_matrix, read_stats)) begin
            $error("appending to read matrix unsuccessful");
            return;
        end
    endfunction

    function automatic void write_burst_start_record2matrix(
        ref write_stats_matrix_t write_matrix
    );
        write_stats_t write_stats;
        write_stats.address         = axi_bus.aw_addr;
        write_stats.time_issued     = $time;
        write_stats.burst_length    = axi_bus.aw_len;
        write_stats.id              = axi_bus.aw_id;
        // bin initial clock cycle, since normal counter operation starts only on the next cycle
        casez ({axi_bus.w_valid, axi_bus.w_ready,
                axi_bus.aw_ready, wr_aw_outstanding == wr_w_outstanding}) //axi_bus.aw_valid is implicitly high since function was called
            4'b11?1: write_stats.beat_count         = MAXCNT'('d1);
            4'b1001: write_stats.early_stall_count  = MAXCNT'('d1);
            4'b1011: write_stats.stall_count        = MAXCNT'('d1);

            4'b1?10,                                                // WVALID doesn't belong to this burst
            4'b0?1?: write_stats.awr_early_count    = MAXCNT'('d1);

            4'b1?00,                                                // WVALID doesn't belong to this burst
            4'b0?0?: write_stats.addr_stall_count   = MAXCNT'('d1);
        endcase
        if (axi_bus.aw_ready) begin
            write_stats.wr_aw_outstanding  = 1'b1;
        end
        // write_matrix[write_stats.id].push_back(write_stats);
        if (!write_matrix_append(write_matrix, write_stats)) begin
            $error("appending to write matrix unsuccessful");
            return;
        end
    endfunction

    function automatic void write_burst_start_record2temp(
        ref temp_write_stats_t temp_stats_arr[MAXWBURSTS]
    );
        temp_write_stats_t temp_write_stats;
        temp_write_stats.time_issued = $time;
        $warning("recording to temp arr started at %d ps", $time);
        // bin initial clock cycle, since normal counter operations start only on the next cycle
        if (axi_bus.w_ready) begin
            temp_write_stats.early_beat_count   = MAXCNT'('d1);
        end else begin
            temp_write_stats.early_stall_count        = MAXCNT'('d1);
        end
        if (axi_bus.w_ready && axi_bus.w_last) begin
            temp_write_stats.wr_w_outstanding   = 1'b1; // if WREADY && WLAST on the same cycle, then we immediately have outstanding write data
            temp_write_stats.wr_in_progress     = 1'b0; // except if WLAST on same cycle
        end else begin
            temp_write_stats.wr_in_progress = 1'b1; // WVALID high, so write is in progress
        end
        // temp_stats_arr.push_back(temp_write_stats);
        if (!temp_write_stats_append(temp_stats_arr, temp_write_stats)) begin
            $error("appending to write matrix unsuccessful");
            return;
        end
    endfunction

    function automatic void write_burst_merge_stats(
        ref write_stats_matrix_t write_matrix,
        ref temp_write_stats_t temp_stats_arr[MAXWBURSTS]
    );
        temp_write_stats_t temp_stats;
        write_stats_t write_stats;

        write_stats.address             = axi_bus.aw_addr;
        write_stats.burst_length        = axi_bus.aw_len;
        write_stats.id                  = axi_bus.aw_id;

        if (!temp_write_stats_pop(temp_stats, temp_stats_arr)) begin
            $error("pop on temp_stats_arr unsuccessful, empty queue");
            return;
        end

        // bin clock cycle during merging, since normal counter operation is suspended to avoid ambiguous behaviour of temp_stats_arr[0]
        casez ({axi_bus.w_valid, axi_bus.w_ready,
                axi_bus.aw_ready, temp_stats.wr_w_outstanding,
                temp_stats.wr_in_progress})
            5'b11?0?: temp_stats.beat_count        += MAXCNT'('d1);
            5'b10001: temp_stats.early_stall_count += MAXCNT'('d1);
            5'b10101: temp_stats.stall_count       += MAXCNT'('d1);
            5'b??010,
            5'b0?00?: temp_stats.addr_stall_count  += MAXCNT'('d1);
            default: merge_wr_no_increment = 1'b1;
        endcase

        write_stats.time_issued         = temp_stats.time_issued;
        write_stats.stall_count         = temp_stats.stall_count;
        write_stats.beat_count          = temp_stats.beat_count;
        write_stats.early_stall_count   = temp_stats.early_stall_count;
        write_stats.early_beat_count    = temp_stats.early_beat_count;
        write_stats.addr_lag_count      = temp_stats.addr_lag_count;
        write_stats.addr_stall_count    = temp_stats.addr_stall_count;

        if (axi_bus.aw_ready) begin
            write_stats.wr_aw_outstanding = 1'b1;
        end
        // transfer status of outstanding data and write in progress to matrix
        write_stats.wr_w_outstanding    = temp_stats.wr_w_outstanding;
        write_stats.wr_in_progress      = temp_stats.wr_in_progress;
        // except if corresponding data & address are on the bus at the same time
        if (axi_bus.w_valid && temp_stats.wr_in_progress) begin
            if (axi_bus.w_ready && axi_bus.w_last) begin // and it is the last data beat of the burst
                write_stats.wr_w_outstanding    = 1'b1;
                write_stats.wr_in_progress      = 1'b0;
                // don't pop_front on wr_wlast_outstanding here, since WLAST is present simultaneous to the address, so there was no entry
                // created for this burst before (since there was no ID available at creation) and no need to do it now, since it is done already
            end else begin
                write_stats.wr_in_progress  = 1'b1;
                // wr_wlast_outstanding.push_back(write_stats.id); // do however create an entry, if write in still in progress at time of merge
                w_channel_token_queue.push_back('{id: write_stats.id, idx: write_matrix_last[write_stats.id]+integer'(!wr_recording_done[write_stats.id])}); // store the ID and index of the burst in the write matrix, note: write_matrix_last indicates the last element in the matrix, so we need to add 1 to it to get the index of the new element, except if a recording finishes in the same cycle
            end
        end
        // write_matrix[write_stats.id].push_back(write_stats);
        if (!write_matrix_append(write_matrix, write_stats)) begin
            $error("appending to write matrix unsuccessful");
            return;
        end
    endfunction

    function automatic void read_burst_record_completion_time(
        ref read_stats_matrix_t read_matrix,
        input int id
    );
        if (id > ((1<<RID_WIDTH) - 1)) begin
            $error("RID=%d out of range of read_matrix IDs", id);
            return;
        end
        if (read_matrix_empty[id]) begin
            $error("no burst in read_matrix currently recording, completion not possible for RID=%d", id);
            return;
        end
        read_matrix[id][0].time_completed = $time;
    endfunction

    function automatic void read_burst_save_recording(
        ref read_stats_t read_results_arr[MAX_NUM_REC_BURSTS],
        ref read_stats_matrix_t read_matrix,
        input int id,
        input logic filter
    );
        read_stats_t read_stats;

        // if (read_matrix_empty[id]) begin
        if(!read_matrix_pop(read_stats, read_matrix, id)) begin
            $error("pop on read_matrix for ID=%d unsuccessful, empty array", id);
            return;
        end
        // prevent storage of transfers from/to filtered address
        if (!filter) begin
            // Read Latency    = (AR Cycles + AR Stalls + RdLag + R Stalls + SlowRdLink)
            read_stats.latency  = read_stats.ar_cycle_count
                                    +  read_stats.ar_stall_count
                                    +  read_stats.lag_count
                                    +  read_stats.r_stall_count
                                    +  read_stats.slow_link_count;
            // Initiation Delay
            read_stats.init_delay   = read_stats.lag_count
                                    +  read_stats.ar_cycle_count
                                    +  read_stats.ar_stall_count;
            // Read Throughput = (Rd beats)/(Rd Beats + R Stalls + RSlow) -> only store denominator, divide in postprocessing
            read_stats.throughput_dnm   = read_stats.beat_count
                                        + read_stats.r_stall_count
                                        + read_stats.slow_link_count;
            // Transfer duration
            read_stats.duration = read_stats.ar_cycle_count
                                + read_stats.ar_stall_count
                                + read_stats.lag_count
                                + read_stats.r_stall_count
                                + read_stats.slow_link_count
                                + read_stats.beat_count;
            read_stats_append(read_results_arr, read_stats);
        end
        // read_stats = '{default: 'b0}; // reset to 0 is very important, so old values don't carry over to the new burst
    endfunction

    function automatic void write_burst_record_completion_time(
        ref write_stats_matrix_t write_matrix,
        input int id
    );
        if (id > ((1<<WID_WIDTH) - 1)) begin
            $error("ID=%d out of range of write_matrix IDs", id);
            return;
        end
        if (write_matrix_empty[id]) begin
            $error("no burst in write_matrix currently recording, completion not possible for BID=%d", id);
            return;
        end
        write_matrix[id][0].time_completed = $time;
    endfunction

    function automatic void write_burst_save_recording(
        ref write_stats_t write_results_arr[MAX_NUM_REC_BURSTS],
        ref write_stats_matrix_t write_matrix,
        input int id,
        input logic filter
    );
        write_stats_t write_stats;

        if(!write_matrix_pop(write_stats, write_matrix, id)) begin
            $error("pop on write_matrix for ID=%d unsuccessful", id);
            return;
        end
        // prevent storage of transfers from/to filtered address
        if (!filter) begin
            // Write Latency =  EarlyWriteAddr + WrAddrStall + WrDataLag + WrAddrLag + EarlyDataStall + B-ChannelLag + B-ChannelStall
            write_stats.latency = write_stats.awr_early_count
                                + write_stats.addr_stall_count
                                + write_stats.early_stall_count
                                + write_stats.data_lag_count
                                + write_stats.addr_lag_count
                                + write_stats.b_lag_count
                                + write_stats.b_stall_count;
            // Write Initiation Delay
            write_stats.init_delay  = write_stats.awr_early_count
                                    + write_stats.addr_stall_count
                                    + write_stats.early_stall_count
                                    + write_stats.data_lag_count;
            // Write Throughput = Wr Beats + Write Early Beats / Wr Beats + Wr Early Beats + WrStalls + WrSlow -> only store denominator, divide in postprocessing
            write_stats.throughput_dnm  = write_stats.beat_count
                                        + write_stats.early_beat_count
                                        + write_stats.slow_data_count
                                        + write_stats.stall_count;
            // Write Transfer Duration
            write_stats.duration    = write_stats.beat_count
                                    + write_stats.early_beat_count
                                    + write_stats.stall_count
                                    + write_stats.slow_data_count
                                    + write_stats.awr_early_count
                                    + write_stats.addr_stall_count
                                    + write_stats.data_lag_count
                                    + write_stats.addr_lag_count
                                    + write_stats.early_stall_count
                                    + write_stats.b_lag_count
                                    + write_stats.b_stall_count
                                    + write_stats.b_end_count;
            // Write Backpressure = Write Latency bis Beginn Datenphase + Write Throughput Zyklen
            write_stats.backpressure    = write_stats.awr_early_count
                                        + write_stats.addr_stall_count
                                        + write_stats.data_lag_count
                                        + write_stats.addr_lag_count
                                        + write_stats.early_beat_count
                                        + write_stats.beat_count
                                        + write_stats.slow_data_count
                                        + write_stats.stall_count
                                        + write_stats.early_stall_count;
            write_stats_append(write_results_arr, write_stats);
        end
        // write_stats = '{default: 'b0};  // reset to 0 is very important, so old values don't carry over to the new burst
    endfunction

//------------------------------------------------------------------------------
//  Ordering of raw data in multidimensional array (tbd)
//------------------------------------------------------------------------------

    /* //definition of the indices
    localparam int SIZES[5] = '{128, 64, 32, 16, 8};
    localparam int MAX_BURST_SIZE = 128;

    typedef enum int {
        8_BIT,
        16_BIT,
        32_BIT,
        64_BIT,
        128_BIT
    } SizeEnum;

    //WIP extend continuously
    typedef enum int {
        TOTAL_BEATS,
        TOTAL_BURSTS,
        MIN_LATENCY,
        MAX_LATENCY,
        AVG_LATENCY,
        MEDIAN_LATENCY
    } MetricEnum;

    int read_data[SizeEnum][0:MAX_BURST_SIZE][MetricEnum];
    int write_data[SizeEnum][0:MAX_BURST_SIZE][MetricEnum];

    initial begin
        //example usage
        read_data[128_BIT][0][AR_STALLS] = 80;
        read_data[8_BIT][MAX_BURST_SIZE-1][LATENCY] = 16;
        read_data[64_BIT][32][LATENCY] = 32;
        read_data[128_BIT][0][BURST_COUNT] = 15;

        $display("AXI4PERFMONITOR_INFO: read_data[size=%s][burst_size=%d][%s]=%d", "128_BIT"   , 0                 , "TOTAL_BEATS",    read_data[128_BIT][0][TOTAL_BEATS]);
        $display("AXI4PERFMONITOR_INFO: read_data[size=%s][burst_size=%d][%s]=%d", "8_BIT"     , MAX_BURST_SIZE-1  , "AVG_LATENCY",    read_data[8_BIT][MAX_BURST_SIZE-1][AVG_LATENCY]);
        $display("AXI4PERFMONITOR_INFO: read_data[size=%s][burst_size=%d][%s]=%d", "64_BIT"    , 32                , "AVG_LATENCY",    read_data[64_BIT][32][AVG_LATENCY]);
        $display("AXI4PERFMONITOR_INFO: read_data[size=%s][burst_size=%d][%s]=%d", "128_BIT"   , 0                 , "TOTAL_BURSTS",   read_data[128_BIT][0][TOTAL_BURSTS]);
    end */

    // Function to set value in the array
    /* function void set_value(int first_index, int second_index, string third_index, int value);
        if (read_data[first_index][second_index].exists(third_index)) begin
            read_data[first_index][second_index][third_index] = value;
        end else begin
            $display("Error: third_index \"%s\" does not exist for array[%0d][%0d]", third_index, first_index, second_index);
        end
    endfunction */

    /* set_value(128, 0, "latency", 42);
    set_value(128, MAX_BURST_SIZE-1, "AR stalls", 43);
    set_value(64, 0, "Avg latency", 100);
    set_value(64, 1, "Burst count", 645); */

    // Iterate through the array and print every value
   /*  initial begin
        int size_index;
        int burst_size_index;
        string metric_index;

        foreach (size_index in SIZES[]) begin
            foreach (burst_size_index from 0 to MAX_BURST_SIZE) begin
                foreach (read_data[size_index][burst_size_index].keyz(metric_index)) begin
                    int value = read_data[size_index][burst_size_index][metric_index];
                    $display("read_data[size=%0d][burst_size=%0d][\"%s\"] = %0d", size_index, burst_size_index, metric_index, value);
                end
            end
        end
    end */

    //2D array for storage of the performance data
    /* localparam DATA_FIELD_SIZE = 5;
    typedef int data_field_t[DATA_FIELD_SIZE];

    typedef struct {
        int row;
        int col;
    } TupleKey;

    // Define the struct type for the SparseArray
    typedef struct {
        TupleKey key;
        data_field_t data;
    } SparseArrayElem;

    // Define the SparseArray type using the struct type
    typedef data_field_t[][TupleKey] SparseArray;

    SparseArray read_data[$];
    SparseArray another_read_data[$];

    // Function to set data entry in the sparse array
    function void set_data(SparseArray sparse_array, int row, int col, data_field_t _data);
        TupleKey key;
        key.row = row;
        key.col = col;
        sparse_array[key] = _data;
    endfunction

    // Function to get data entry from the sparse array
    function data_field_t get_data(SparseArray sparse_array, int row, int col);
        TupleKey key;
        key.row = row;
        key.col = col;

        data_field_t _data;

        if (sparse_array.exists(key)) begin
            _data = sparse_array[key];
        end

        return _data;
    endfunction */

//------------------------------------------------------------------------------
//  Assertion Parameters
//------------------------------------------------------------------------------
    localparam READ_COUNTER_ADDUP_OFFSET = 2;
    localparam WRITE_COUNTER_ADDUP_OFFSET = 2;

    default clocking clk_default @(posedge clk);
    endclocking

    default disable iff (!nres);

//-------------------------------------------------------------------------------------------------------------------------
//  Credits for the calculation of the performance metrics Read/Write Latency, Read/Write Throughput,
//  the Read/Write clock cycle bins, module control signals for starting/stopping the measurement, wr_w/aw_outstanding
//  counters go to:
//      Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//      https://github.com/ZipCPU/wb2axip/blob/master/rtl/axiperf.v
//
//  In compliance with the Apache License 2.0 this code is a derivative of the original source code. You may obtain a copy
//  of the License at http://www.apache.org/licenses/LICENSE-2.0
//-------------------------------------------------------------------------------------------------------------------------

//------------------------------------------
//  Control logic
//------------------------------------------

always_comb begin : ctrl_signal_cond_assign
    if (!perf_err) begin
        if (MANUAL_CONTROL) begin
            assign start_measurement    = manual_start;
            assign stop_measurement     = manual_stop;
            assign clear_measurement    = manual_clear;
            assign print_measurement    = manual_print;
        end else begin
            assign start_measurement    = ~stopclock;
            assign stop_measurement     = stopclock;
            assign clear_measurement    = 1'b0;
            assign print_measurement    = stopclock;
        end
    end else begin  // if error occurs, print out the currently recorded stats and set all other control signals to 0
        start_measurement   = 1'b0;
        stop_measurement    = 1'b0;
        clear_measurement   = 1'b0;
        print_measurement   = 1'b1;
    end
end

// Assertions belonging to the control signals, aimed at informing the user of this module if the order of signaling is wrong
property clear_simultaneous_to_start;
    !(clear_measurement && start_measurement);
endproperty
assert property (clear_simultaneous_to_start)
else $error("clear_measurement asserted simultaneous to start_measurement");

// flag that indicates start_measurement was high since reset
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        start_rcvd  <= 1'b0;
    end else begin
        if (start_measurement) begin
			start_rcvd  <= 1'b1;
        end
        if (stop_measurement) begin
            start_rcvd  <= 1'b0;
        end
    end
end

// trigger for read recording, goes high when the first ARVALID occurs after start_measurement was set
// goes low on the posedge of stopclock or when stop_measurement is set
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        r_rd_rec_triggered  <= 1'b0;
        rd_first_in_rec     <= 1'b1;
    end else if (perf_err) begin
        r_rd_rec_triggered  <= 1'b0;
        rd_first_in_rec     <= 1'b0;
    end else if (stop_measurement) begin
        r_rd_rec_triggered  <= 1'b0;
        rd_first_in_rec     <= 1'b1;
    end else if (axi_bus.ar_valid && (start_rcvd || start_measurement) && rd_channel_idle_reg) begin
        if (rd_first_in_rec) begin
            r_rd_rec_triggered    <= 1'b1;
        end
        rd_first_in_rec <= 1'b0;
    end
end

assign rd_rec_triggered = (r_rd_rec_triggered || (axi_bus.ar_valid && rd_first_in_rec && (start_rcvd || start_measurement) && rd_channel_idle_reg)) && !stopclock;

// trigger for write recording, goes high when the first AWVALID/WVALID occurs after start_measurement was set
// goes low on the posedge of stopclock or when stop_measurement is set
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        r_wr_rec_triggered  <= 1'b0;
        wr_first_in_rec     <= 1'b1;
    end else if (perf_err) begin
        r_wr_rec_triggered  <= 1'b0;
        wr_first_in_rec     <= 1'b0;
    end else if (stop_measurement) begin
        r_wr_rec_triggered  <= 1'b0;
        wr_first_in_rec     <= 1'b1;
    end else if ((axi_bus.aw_valid || axi_bus.w_valid) && (start_rcvd || start_measurement) && wr_channel_idle_reg) begin
        if (wr_first_in_rec) begin
            r_wr_rec_triggered    <= 1'b1;
        end
        wr_first_in_rec <= 1'b0;
    end
end
assign wr_rec_triggered = (r_wr_rec_triggered || ((axi_bus.aw_valid || axi_bus.w_valid) && (start_rcvd || start_measurement) && wr_channel_idle_reg)) && !stopclock;

//counter that captures the total active measurement time
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        active_count        <= 'b0;
        active_count_err    <= 1'b0;
    end
    else if (rd_rec_triggered || wr_rec_triggered) begin
        if (!active_count[MAXCNT-1])  begin     //as long as the counter is not overflowing continue counting
            active_count <= active_count + MAXCNT'('d1);
        end else begin          //counter overflow detected, stop the measurement preemptively and display an error
            active_count_err <= 1'b1;
            $error("AXI4PERFMONITOR: Active Measurement Time larger than expected, stopping measurement at cycle %d", active_count);
        end
    end
end

// counter that counts total idle cycles
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        idle_count <= 0;
    end else if ((rd_rec_triggered && rd_channel_idle) || (wr_rec_triggered && wr_channel_idle)) begin
        idle_count <= idle_count + MAXCNT'('d1);
    end
end

/* // logic to monitor if bus is in idle state
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        r_idle_bus <= 1'b1;
    end else if (perf_err) begin
       r_idle_bus <= 1'b1;
    end else if (axi_bus.aw_valid || axi_bus.w_valid || axi_bus.ar_valid) begin
        r_idle_bus <= 1'b0;
    end else if ((wr_aw_outstanding == ((axi_bus.b_valid && axi_bus.b_ready) ? 1:0))
            && (wr_w_outstanding == ((axi_bus.b_valid && axi_bus.b_ready) ? 1:0))
            && (rd_ar_outstanding == ((axi_bus.r_valid && axi_bus.r_ready && axi_bus.r_last) ? 1:0))) begin
        r_idle_bus <= 1'b1;
    end
    assign	idle_bus = r_idle_bus && !axi_bus.aw_valid && !axi_bus.w_valid
            && !axi_bus.ar_valid;
end */

// flag that indicates if read channel is idle
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        r_rd_channel_idle   <= 1'b1;
        rd_channel_idle_reg	<= 1'b1;
    end else if (perf_err) begin
        r_rd_channel_idle   <= 1'b1;
    end else if (axi_bus.ar_valid) begin
        r_rd_channel_idle   <= 1'b0;
    end else if (rd_ar_outstanding == ((axi_bus.r_valid && axi_bus.r_ready && axi_bus.r_last) ? 1:0)) begin
        r_rd_channel_idle   <= 1'b1;
    end
    assign rd_channel_idle  = r_rd_channel_idle && !axi_bus.ar_valid;
    rd_channel_idle_reg     <= rd_channel_idle;
end

// flag that indicates that write channel is idle
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        r_wr_channel_idle   <= 1'b1;
        wr_channel_idle_reg <= 1'b1;
    end else if (perf_err) begin
        r_wr_channel_idle   <= 1'b1;
    end else if (axi_bus.aw_valid || axi_bus.w_valid) begin
        r_wr_channel_idle   <= 1'b0;
    end else if ((wr_aw_outstanding == ((axi_bus.b_valid && axi_bus.b_ready) ? 1:0))
            && (wr_w_outstanding == ((axi_bus.b_valid && axi_bus.b_ready) ? 1:0))) begin
        r_wr_channel_idle   <= 1'b1;
    end
    assign wr_channel_idle  = r_wr_channel_idle && !axi_bus.aw_valid && !axi_bus.w_valid;
    wr_channel_idle_reg     <= wr_channel_idle;
end

// if any of the overflow bits of a burst tracking counter is high, this indicates an unrecoverable error of the perfmonitor requiring reset
always @(posedge clk or negedge nres) begin
    if (!nres) begin
        perf_err <= 1'b0;
    end
    else if (wr_aw_err || wr_w_err || rd_err || wr_awvalid_err || wr_wvalid_err || active_count_err) begin
        perf_err <= 1'b1;
        $error("Perfmonitor Error occured, reset required before module may be used again! %h", {wr_aw_err, wr_w_err, rd_err, wr_awvalid_err, wr_wvalid_err, active_count_err});
    end
end

//------------------------------------------
//  Read measurement logic
//------------------------------------------

// flag that indicates if ARVALID already occured for this burst
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        arvalid_occured    <= 1'b0;
    end else if (axi_bus.ar_valid) begin
        if (axi_bus.ar_ready) begin
            arvalid_occured    <= 1'b0;
        end else begin
            arvalid_occured    <= 1'b1;
        end
    end
end

wire first_arvalid;
wire last_rvalid;
assign first_arvalid = axi_bus.ar_valid && !arvalid_occured;
assign last_rvalid = axi_bus.r_valid && axi_bus.r_ready && axi_bus.r_last;

// logic [3:0] read_dummy;
// Management of read burst recording
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        rd_outstanding_bursts       <= 'b0;
        rd_zero_outstanding         <= 1'b1;
        rd_err                      <= 1'b0;
        arvalid_occured           <= 1'b0;
        foreach(read_stats_matrix[id]) begin
            read_stats_matrix[id]   <= '{default: 'b0};
            read_matrix_last[id]    <= -1;
            rd_bursts_in_flight[id] <= 1'b0;
        end
        read_matrix_empty           <= '{default: 1'b1};
        read_matrix_full            <= '{default: 1'b0};
        read_stats_arr              <= '{default: 'b0};
        read_stats_last             <= -1;
        r_rd_total_r_stall_count    <= 'b0;
        r_rd_total_slow_link_count  <= 'b0;
        r_rd_total_lag_count        <= 'b0;
        r_rd_total_ar_stall_count   <= 'b0;
        rd_recording_done           <= 'b0;
        rd_recording_saved          <= 1'b0;
        // read_dummy                  <= 'b0;
    end
    else if (rd_rec_triggered && !rd_err) begin
        rd_recording_done <= 'b0;
        casez ({first_arvalid, last_rvalid})
            2'b10: begin
                // read_dummy <= 4'h1;
                read_burst_start_record2matrix(read_stats_matrix);
                // update outstanding counter/flag
                {rd_err, rd_outstanding_bursts} <= rd_outstanding_bursts + MAXRBURSTS_WIDTH'('d1);
                rd_zero_outstanding <= 1'b0;
            end
            2'b01: begin
                // read_dummy <= 4'h2;
                read_burst_record_completion_time(read_stats_matrix, axi_bus.r_id);
                rd_recording_done[axi_bus.r_id] <= 1'b1;
                // update outstanding counter/flag
                {rd_err, rd_outstanding_bursts} <= rd_outstanding_bursts - MAXRBURSTS_WIDTH'('d1);
                rd_zero_outstanding <= (rd_outstanding_bursts == MAXRBURSTS_WIDTH'('d1)? 1'b1 : 1'b0);
            end
            2'b11: begin
                // read_dummy <= 4'h3;
                read_burst_start_record2matrix(read_stats_matrix);
                read_burst_record_completion_time(read_stats_matrix, axi_bus.r_id);
                rd_recording_done[axi_bus.r_id] <= 1'b1;
                // don't update counter, since increment and decrement by one in same cycle lead to net change of 0
            end
            default: begin
                // read_dummy <= 4'h4;
            end
        endcase

        // update rd_ar_outstanding flag in matrix when ARREADY goes high not in the first ARVALID cycle, since then it is handled in read_burst_start_record2matrix function
        if (axi_bus.ar_valid && axi_bus.ar_ready && arvalid_occured) begin
            // if recording for this id is done at the same time as update of rd_ar_outstanding is needed, choose the second to last element for this ID, since that contains the last element after shifting in read_matrix_pop()
            read_stats_matrix[axi_bus.ar_id][read_matrix_last[axi_bus.ar_id]-integer'(rd_recording_done[axi_bus.ar_id])].rd_ar_outstanding <= 1'b1;
        end

        if (axi_bus.r_valid) begin
			if (axi_bus.r_ready && axi_bus.r_last) begin
                read_stats_matrix[axi_bus.r_id][0].rd_ar_outstanding   <= 1'b0;
                rd_bursts_in_flight[axi_bus.r_id]   <= 1'b0;
            end else begin
				rd_bursts_in_flight[axi_bus.r_id]   <= 1'b1;
            end
        end

        r_rd_total_r_stall_count    <= 0;
        r_rd_total_slow_link_count  <= 0;
        r_rd_total_lag_count        <= 0;
        r_rd_total_ar_stall_count   <= 0;

        rd_recording_saved          <= 1'b0;
        foreach (rd_recording_done[id]) begin
            if (rd_recording_done[id]) begin
                // filter out transfers that match address filter
                if (!FILTER_EN || (FILTER_EN && (read_stats_matrix[id][0].address & FILTER_MASK) != (FILTER_ADDR & FILTER_MASK))) begin
                    r_rd_total_r_stall_count    <= read_stats_matrix[id][0].r_stall_count;    // only one read burst can finish each cycle, therefore assignments are not ambiguous
                    r_rd_total_slow_link_count  <= read_stats_matrix[id][0].slow_link_count;
                    r_rd_total_lag_count        <= read_stats_matrix[id][0].lag_count;
                    r_rd_total_ar_stall_count   <= read_stats_matrix[id][0].ar_stall_count;
                    rd_recording_saved          <= 1'b1;
                    read_burst_save_recording(read_stats_arr, read_stats_matrix, id, 1'b0);
                end else begin
                    read_burst_save_recording(read_stats_arr, read_stats_matrix, id, 1'b1);
                end
            end
        end
    end
end

always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        rd_no_increment <= '{default: 'b0};
    end else if (rd_rec_triggered) begin
        rd_no_increment <= '{default: 'b0};
        foreach(read_stats_matrix[id]) begin
            for (int idx=0; idx<(read_matrix_last[id]+1); idx++) begin  // loop only over all active elements of this ID
                if (rd_recording_done[id] && idx==0) begin
                    continue;
                end else begin   // prevent counter updates of recording at index 0 if recording is already done, might leak into next burst statistics otherwise
                    casez({ read_stats_matrix[id][idx].rd_ar_outstanding, rd_bursts_in_flight[id],
                            axi_bus.ar_valid, axi_bus.ar_ready, axi_bus.ar_id==id,
                            axi_bus.r_valid, axi_bus.r_ready, axi_bus.r_id==id,
                            idx==(0+integer'(rd_recording_done[id])) }) // rd_recording_done[id] being asserted, means the recording in index 0 is done for this ID, therefore the new head of the recording queue is at index 1 -> a beat in this cycle needs to be attributed to it index 1, and everything else is shifted down in the matrix by one
                        9'b1?_???_111_1:   read_stats_matrix[id][idx-integer'(rd_recording_done[id])].beat_count        <= read_stats_matrix[id][idx].beat_count         + MAXCNT'('d1); // Read beat
                        9'b1?_???_101_1:   read_stats_matrix[id][idx-integer'(rd_recording_done[id])].r_stall_count     <= read_stats_matrix[id][idx].r_stall_count      + MAXCNT'('d1); // Read stall
                        9'b11_???_0?1_1:   read_stats_matrix[id][idx-integer'(rd_recording_done[id])].slow_link_count   <= read_stats_matrix[id][idx].slow_link_count    + MAXCNT'('d1); // Slow read link
                        9'b1?_???_???_0,
                        9'b10_???_0??_1,
                        9'b10_???_1?0_1:   read_stats_matrix[id][idx-integer'(rd_recording_done[id])].lag_count         <= read_stats_matrix[id][idx].lag_count          + MAXCNT'('d1); // Read lag counter
                        9'b0?_101_???_?:   read_stats_matrix[id][idx-integer'(rd_recording_done[id])].ar_stall_count    <= read_stats_matrix[id][idx].ar_stall_count     + MAXCNT'('d1); // Read address stall
                        9'b0?_111_???_?:   read_stats_matrix[id][idx-integer'(rd_recording_done[id])].ar_cycle_count    <= read_stats_matrix[id][idx].ar_cycle_count     + MAXCNT'('d1); // Read address cycle
                        default:        rd_no_increment[id][idx] <= 1'b1;
                    endcase
                end
            end
        end
    end
end

// Cumulation of total read counter values
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        rd_total_beat_count             <= 0;
        rd_total_burst_count            <= 0;
        rd_total_ar_stall_count         <= 0;
        rd_total_r_stall_count          <= 0;
        rd_total_bus_active_count       <= 0;
        rd_total_arvalid_count          <= 0;
        rd_total_rvalid_count           <= 0;
        rd_total_idle_count             <= 0;
        rd_total_ar_cycle_count         <= 0;
        rd_total_slow_link_count        <= 0;
        rd_total_lag_count              <= 0;
        rd_total_in_flight              <= 0;
    end
    if (rd_rec_triggered) begin
        if (axi_bus.r_valid && axi_bus.r_ready) begin
            rd_total_beat_count     <= rd_total_beat_count + MAXCNT'('d1);
        end
        if (axi_bus.ar_valid && axi_bus.ar_ready) begin
            rd_total_burst_count    <= rd_total_burst_count + MAXCNT'('d1);
        end
        if (axi_bus.ar_valid && axi_bus.ar_ready) begin
            rd_total_ar_cycle_count <= rd_total_ar_cycle_count + MAXCNT'('d1);
        end
        if (axi_bus.ar_valid) begin
            rd_total_arvalid_count  <= rd_total_arvalid_count + MAXCNT'('d1);
        end
        if (axi_bus.r_valid) begin
            rd_total_rvalid_count   <= rd_total_rvalid_count + MAXCNT'('d1);
        end
        if (rd_channel_idle) begin
            rd_total_idle_count     <= rd_total_idle_count + MAXCNT'('d1);
        end else begin
            rd_total_bus_active_count   <= rd_total_bus_active_count + MAXCNT'('d1);
        end
        rd_total_ar_stall_count     <= rd_total_ar_stall_count + r_rd_total_ar_stall_count;
        rd_total_r_stall_count      <= rd_total_r_stall_count + r_rd_total_r_stall_count;
        rd_total_slow_link_count    <= rd_total_slow_link_count + r_rd_total_slow_link_count;
        rd_total_lag_count          <= rd_total_lag_count + r_rd_total_lag_count;
        rd_total_in_flight          <= $countones(rd_bursts_in_flight);
    end
end

// Read total outstanding bursts
always @(posedge clk or negedge nres) begin
    if (!nres) begin
        rd_ar_outstanding           <= 'b0;
        rd_ar_nonzero_outstanding   <= 1'b0;
        rd_ar_err                   <= 1'b0;
    end else if (!rd_ar_err) begin
        case ({axi_bus.ar_valid && axi_bus.ar_ready,
                    axi_bus.r_valid && axi_bus.r_ready && axi_bus.r_last})
            2'b10: begin
                {rd_ar_err, rd_ar_outstanding}  <= rd_ar_outstanding + MAXRBURSTS_WIDTH'('d1);
                rd_ar_nonzero_outstanding       <= 1'b1;
            end
            2'b01: begin
                {rd_ar_err, rd_ar_outstanding}  <= rd_ar_outstanding - MAXRBURSTS_WIDTH'('d1);
                rd_ar_nonzero_outstanding       <= (rd_ar_outstanding == MAXRBURSTS_WIDTH'('d1)) ? 1'b0 : 1'b1;
            end
            default: begin end
        endcase
    end
end

// max read outstanding bursts
always_ff @( posedge clk or negedge nres ) begin
    if (!nres || clear_measurement) begin
        rd_max_outstanding <= 0;
    end else if (rd_rec_triggered) begin
        if (rd_ar_outstanding > rd_max_outstanding) begin
            rd_max_outstanding <= rd_ar_outstanding;
        end
    end
end

/* //Read Lag for first request coming from idle bus (yet to be used)
always @(posedge clk or negedge nres) begin
    if (!nres) begin
        rd_first <= 1'b1;
    end else if (axi_bus.r_valid) begin
        rd_first <= 1'b0;
    end else if (axi_bus.ar_valid && !rd_ar_nonzero_outstanding) begin
        rd_first <= 1'b1;
    end
end

always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        rd_first_lag <= 0;
    end else if (rd_rec_triggered && rd_first) begin
        rd_first_lag <= rd_first_lag + MAXCNT'('d1);
    end
end */

//------------------------------------------
//  Write measurement logic
//------------------------------------------

// flag that indicates if AWVALID already occured for this burst
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        awvalid_occured    <= 1'b0;
    end else if (axi_bus.aw_valid) begin
        if (axi_bus.aw_ready) begin
            awvalid_occured    <= 1'b0;
        end else begin
            awvalid_occured    <= 1'b1;
        end
    end
end

wire first_awvalid;
assign first_awvalid = axi_bus.aw_valid && !awvalid_occured;

wire first_wvalid;
assign first_wvalid = axi_bus.w_valid && !wvalid_occured;

// awvalid/wvalid_unmatched counters indicate how many first AWVALID/WVALID cycles of a burst have occured without the corresponding
// first WVALID/AWVALID cycle
// (refer to ../doc/awvalid_unmatched_timing_diagram.png and ../doc/wvalid_unmatched_timing_diagram.png)
logic [3:0] write_dummy;
token_t ret_token;
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        write_stats_arr         <= '{default: 'b0};
        write_stats_last        <= -1;
        // wr_wlast_outstanding.delete();
        w_channel_token_queue.delete();
        ret_token               <= '{default: 'b0};
        temp_write_stats_arr    <= '{default: 'b0};
        temp_write_stats_last   <= -1;
        temp_write_stats_empty  <= 1'b1;
        temp_write_stats_full   <= 1'b0;
        foreach(write_stats_matrix[id]) begin
            write_stats_matrix[id]      <= '{default: 'b0};
            write_matrix_last[id]       <= -1;
        end
        write_matrix_empty              <= '{default: 1'b1};
        write_matrix_full               <= '{default: 1'b0};
        wr_recording_done               <= 'b0;
        wr_recording_saved              <= 1'b0;
        first_unmatched_wlast_idx       <= 0;
        r_wr_total_addr_stall_count     <= 'b0;
        r_wr_total_addr_lag_count       <= 'b0;
        r_wr_total_early_stall_count    <= 'b0;
        r_wr_total_b_lag_count          <= 'b0;
        r_wr_total_b_stall_count        <= 'b0;
        r_wr_total_data_lag_count       <= 'b0;
        r_wr_total_stall_count          <= 'b0;
        r_wr_total_slow_data_count      <= 'b0;
        r_wr_total_awr_early_count      <= 'b0;
        r_wr_total_early_beat_count     <= 'b0;
        write_dummy                     <= 'b0;
        temp_stats_created              <= 1'b0;
        wr_wvalid_unmatched         <= 'b0;
        wr_wvalid_zero_unmatched    <= 1'b1;
        wr_wvalid_err               <= 1'b0;
        wr_awvalid_unmatched	    <= 'b0;
        wr_awvalid_zero_unmatched   <= 1'b1;
        wr_awvalid_err              <= 1'b0;
        merge_wr_no_increment       <= 1'b0;
    end else if (wr_rec_triggered && !wr_awvalid_err && !wr_wvalid_err && !wr_awvalid_err && !wr_wvalid_err) begin
        // decision making process if temp write stats needed or stats can be written directly into the 2D write_stats_matrix, also check if outstanding flags need updating
        // (refer to ../doc/Entscheidungsfluss_Aufzeichungsbeginn_Perfmonitor.png) -> outdated, not complete anymore!
        merge_wr_no_increment <= 1'b0;  // set fault flag to 0, is set in write_burst_merge_stats if fault during merge
        casez ({axi_bus.aw_valid, first_awvalid,
                axi_bus.w_valid, wr_in_progress,
                wr_awvalid_zero_unmatched, wr_wvalid_zero_unmatched})
            //first AWVALID of burst precedes first WVALID of burst
            6'b11_00_?1: begin // no data unmatched to address on the bus and no data simultaneous to address on the bus, write directly to matrix
                write_dummy <= 4'h1;
                {wr_awvalid_err, wr_awvalid_unmatched} <= wr_awvalid_unmatched + MAXWBURSTS_WIDTH'('d1);
                wr_awvalid_zero_unmatched   <= 1'b0;
                write_burst_start_record2matrix(write_stats_matrix);
                w_channel_token_queue.push_back('{id: axi_bus.aw_id, idx: write_matrix_last[axi_bus.aw_id]});
            end
            6'b11_00_10, 6'b11_?1_10: begin // first AWVALID and data unmatched to address on bus and no first WVALID simultaneous on the bus, merge temp stats into matrix element
                write_dummy <= 4'h2;
                {wr_wvalid_err, wr_wvalid_unmatched} <= wr_wvalid_unmatched  - MAXWBURSTS_WIDTH'('d1);
                wr_wvalid_zero_unmatched    <= (wr_wvalid_unmatched == MAXWBURSTS_WIDTH'('d1)? 1'b1 : 1'b0);
                write_burst_merge_stats(write_stats_matrix, temp_write_stats_arr);
            end
            // first AWVALID and WVALID (first or not) simultaneous
            6'b11_1?_?1: begin  // no data unmatched to address on the bus and also data simultaneous to address on the bus, write directly to matrix
                // start new burst first and append its ID to outstanding WLAST queue (blocking!)
                write_dummy <= 4'h3;
                if (!first_wvalid) begin   // only increase the awvalid unmatched counter if it is not a first WVALID
                    {wr_awvalid_err, wr_awvalid_unmatched} <= wr_awvalid_unmatched + MAXWBURSTS_WIDTH'('d1);
                    wr_awvalid_zero_unmatched   <= 1'b0;
                end
                write_burst_start_record2matrix(write_stats_matrix);
                w_channel_token_queue.push_back('{id: axi_bus.aw_id, idx: write_matrix_last[axi_bus.aw_id]});
            end
            // first AWVALID and first WVALID simultaneous to unmatched data on bus
            6'b11_10_10: begin // data unmatched to address on bus and also data simultaneous to address on the bus, merge temp stats of unmatched data into matrix, also create temp stats for current data
                write_dummy <= 4'h4;
                write_burst_merge_stats(write_stats_matrix, temp_write_stats_arr);
                write_burst_start_record2temp(temp_write_stats_arr);
            end
            // first WVALID of burst precedes first AWVALID of burst
            6'b0?_10_1?, 6'b10_10_11: begin // no address unmatched to data on bus and no address simultaneous to data on the bus, create temporary write stats
                write_dummy <= 4'h5;
                temp_stats_created  <= 1'b1;
                {wr_wvalid_err, wr_wvalid_unmatched} <= wr_wvalid_unmatched + MAXWBURSTS_WIDTH'('d1);
                wr_wvalid_zero_unmatched    <= 1'b0;
                write_burst_start_record2temp(temp_write_stats_arr);
            end
            // no new burst or merge, write_stats_matrix flags might need updating
            6'b0?_10_01, 6'b10_10_01: begin // first WVALID, address unmatched to data on the bus and no first AWVALID simultaneous to data on the bus, no new burst needed, burst is already in recording
                write_dummy <= 4'h6;
                {wr_awvalid_err, wr_awvalid_unmatched} <= wr_awvalid_unmatched - MAXWBURSTS_WIDTH'('d1);
                wr_awvalid_zero_unmatched   <= (wr_awvalid_unmatched == MAXWBURSTS_WIDTH'('d1)? 1'b1 : 1'b0);
            end
            // no new burst or merge, temp_write_stats flags might need updating
            6'b0?_11_10, 6'b10_11_10: begin // WVALID but not the first neither a first AWVALID of any burst, no unmatched address but unmatched data
                write_dummy <= 4'h7;
                if (axi_bus.w_ready && axi_bus.w_last) begin
                    temp_write_stats_arr[temp_write_stats_last].wr_w_outstanding    <= 1'b1;    // last index is sufficient here, since temp_write_stats_arr is only a vector and no new write burst
                    temp_write_stats_arr[temp_write_stats_last].wr_in_progress      <= 1'b0;    // can be appended before the current last one has had its WLAST
                end else begin
                    temp_write_stats_arr[temp_write_stats_last].wr_in_progress      <= 1'b1;
                end
            end
            default: begin
                write_dummy <= 4'h9;
            end
        endcase

        // update the write address outstanding flag in the matrix when AWREADY goes high not in the first AWVALID cycle, since then it is handled in write_burst_start_record2matrix function
        if (axi_bus.aw_valid && axi_bus.aw_ready && awvalid_occured) begin
            write_stats_matrix[axi_bus.aw_id][write_matrix_last[axi_bus.aw_id]-integer'(wr_recording_done[axi_bus.aw_id])].wr_aw_outstanding <= 1'b1;
        end

        // when a WLAST is received and no recordings are in the temp_write_stats_arr and no merge cycle is happening, pop the first element of the W channel token queue, as anything happening on the W channel is no longer relevant for this burst
        if (axi_bus.w_valid && temp_write_stats_last==-1 && !merge_cycle) begin
            if (axi_bus.w_ready && axi_bus.w_last) begin    // if WLAST is on the bus, update the flags of the oldest burst that still has WLAST outstanding
                write_stats_matrix[w_channel_token_queue[0].id][w_channel_token_queue[0].idx-integer'(wr_recording_done[w_channel_token_queue[0].id])].wr_w_outstanding    <= 1'b1; //idx needs to be reduced by one if recording done for this ID during this cycle to avoid leaving artifacts while a shift happens
                write_stats_matrix[w_channel_token_queue[0].id][w_channel_token_queue[0].idx-integer'(wr_recording_done[w_channel_token_queue[0].id])].wr_in_progress      <= 1'b0; //idx needs to be reduced by one if recording done for this ID during this cycle to avoid leaving artifacts while a shift happens
                if (w_channel_token_queue.size() == 0) begin
                    $error("pop called on empty w_channel_token_queue");
                end
                ret_token <= w_channel_token_queue.pop_front();
            end else begin  // if WLAST is not on the bus, update the flags of the oldest burst that still has WLAST outstanding
                write_stats_matrix[w_channel_token_queue[0].id][w_channel_token_queue[0].idx-integer'(wr_recording_done[w_channel_token_queue[0].id])].wr_in_progress <= 1'b1;  //idx needs to be reduced by one if recording done for this ID during this cycle to avoid leaving artifacts while a shift happens
            end
        end

        // when the write response is received, raise the corresponding flag for BID, record the completion time and reset the outstanding flags of the appropriate burst
        wr_recording_done <= 'b0;
        if (axi_bus.b_valid && axi_bus.b_ready) begin
            write_burst_record_completion_time(write_stats_matrix, axi_bus.b_id);
            write_stats_matrix[axi_bus.b_id][0].wr_aw_outstanding   <= 1'b0;
            write_stats_matrix[axi_bus.b_id][0].wr_w_outstanding    <= 1'b0;
            wr_recording_done[axi_bus.b_id] <= 1'b1;
        end

        // when saving the burst, transfer all metrics that can only be counted per burst to the total counters
        r_wr_total_addr_stall_count     = 'b0;
        r_wr_total_addr_lag_count       = 'b0;
        r_wr_total_early_stall_count    = 'b0;
        r_wr_total_b_lag_count          = 'b0;
        r_wr_total_b_stall_count        = 'b0;
        r_wr_total_data_lag_count       = 'b0;
        r_wr_total_stall_count          = 'b0;
        r_wr_total_slow_data_count      = 'b0;
        r_wr_total_awr_early_count      = 'b0;
        r_wr_total_early_beat_count     = 'b0;
        wr_recording_saved              <= 1'b0;
        foreach (wr_recording_done[id]) begin
            if (wr_recording_done[id]) begin
                // filter out transfers that match address filter
                if (!FILTER_EN || (FILTER_EN && (write_stats_matrix[id][0].address & FILTER_MASK) != (FILTER_ADDR & FILTER_MASK))) begin
                    r_wr_total_addr_stall_count     = write_stats_matrix[id][0].addr_stall_count;
                    r_wr_total_addr_lag_count       = write_stats_matrix[id][0].addr_lag_count;
                    r_wr_total_early_stall_count    = write_stats_matrix[id][0].early_stall_count;
                    r_wr_total_b_lag_count          = write_stats_matrix[id][0].b_lag_count;
                    r_wr_total_b_stall_count        = write_stats_matrix[id][0].b_stall_count;
                    r_wr_total_data_lag_count       = write_stats_matrix[id][0].data_lag_count;
                    r_wr_total_stall_count          = write_stats_matrix[id][0].stall_count;
                    r_wr_total_slow_data_count      = write_stats_matrix[id][0].slow_data_count;
                    r_wr_total_awr_early_count      = write_stats_matrix[id][0].awr_early_count;
                    r_wr_total_early_beat_count     = write_stats_matrix[id][0].early_beat_count;
                    wr_recording_saved              <= 1'b1;
                    write_burst_save_recording(write_stats_arr, write_stats_matrix, id, 1'b0);
                end else begin
                    write_burst_save_recording(write_stats_arr, write_stats_matrix, id, 1'b1);
                end
            end
        end
    end
end


// per Write-burst clock cycle binning
assign merge_cycle = axi_bus.aw_valid && !awvalid_occured && wr_awvalid_zero_unmatched && !wr_wvalid_zero_unmatched;

always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        wr_no_increment         <= '{default: 'b0};
        temp_wr_no_increment    <= 'b0;
    end else if (wr_rec_triggered) begin
        wr_no_increment <= '{default: 'b0};
        foreach (write_stats_matrix[id]) begin
            for (int idx=0; idx<(write_matrix_last[id]+1); idx++) begin // loop only over all active elements for this ID
                if (wr_recording_done[id] && idx==0) begin   // prevent counter updates if recording is already done, otherwise it might leak into next burst statistics
                    continue;
                end
                casez({write_stats_matrix[id][idx].wr_aw_outstanding, write_stats_matrix[id][idx].wr_w_outstanding, write_stats_matrix[id][idx].wr_in_progress,
                        axi_bus.aw_valid, axi_bus.aw_ready,
                        w_channel_token_queue[0].id == id && w_channel_token_queue[0].idx == idx,   // what happens on the W channel belongs to this ID and index
                        axi_bus.w_valid, axi_bus.w_ready,
                        axi_bus.b_valid, axi_bus.b_ready,
                        axi_bus.b_id==id, idx==0+integer'(wr_recording_done[id])}) // wr_recording_done[id] being asserted, means the recording in index 0 is done for this ID, therefore the new head of the recording queue is at index 1 -> a b_end_count in this cycle needs to be attributed to index 1, and everything else is shifted down in the matrix by one
                    // are early beats even possible for writes in the regular matrix? requires aw_outstanding==1'b0 and AWVALID==1'b0 and WVALID==1'b1 && WREADY==1'b1 -> no! this case creates a recording in temp_write_stats_arr
                    // 12'b001_0?_?11_??_?1: write_stats_matrix[id][idx-integer'(wr_recording_done[id])].early_beat_count   <= write_stats_matrix[id][idx].early_beat_count  + MAXCNT'('d1); // Early write beat (Before AWVALID) of burst-appropiate data
                    // Throughput measures
                    12'b?0?_??_111_??_??:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].beat_count         <= write_stats_matrix[id][idx].beat_count          + MAXCNT'('d1); // Write Beat of burst-appropriate data
                    12'b?01_??_10?_??_??:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].slow_data_count    <= write_stats_matrix[id][idx].slow_data_count     + MAXCNT'('d1); // Slow burst-appropiate write data
                    12'b?0?_??_110_??_??:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].stall_count        <= write_stats_matrix[id][idx].stall_count         + MAXCNT'('d1);	// Stall #2 of burst-appropiate data
                    // Lag measures
                    12'b000_11_0??_??_??,
                    12'b000_11_10?_??_??:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].awr_early_count    <= write_stats_matrix[id][idx].awr_early_count     + MAXCNT'('d1); // Early write address
                    12'b000_10_0??_??_??,
                    12'b000_10_10?_??_??:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].addr_stall_count   <= write_stats_matrix[id][idx].addr_stall_count    + MAXCNT'('d1); // Write address stall
                    
                    // Lag measures
                    12'b100_??_0??_??_??,                                                                                                                                                   // W channel is not even relevant to this burst yet, but address was already sent -> Write data lag
                    12'b100_??_10?_??_??:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].data_lag_count     <= write_stats_matrix[id][idx].data_lag_count      + MAXCNT'('d1); // Write data lag
                    12'b010_??_???_??_??,                                                                                                                                                   // Write address lag #1
                    12'b001_??_10?_??_??:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].addr_lag_count     <= write_stats_matrix[id][idx].addr_lag_count      + MAXCNT'('d1); // Write address lag #2

                    // early stall measure
                    12'b000_??_110_??_??:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].early_stall_count  <= write_stats_matrix[id][idx].early_stall_count    + MAXCNT'('d1); // Early Stall

                    // B-channel measures
                    12'b11?_??_???_0?_??,   // response awaited, but BVALID=0 or ...
                    12'b11?_??_???_1?_0?,   // response awaited, BAVLID=1, but BID doesn't match or ...
                    12'b11?_??_???_1?_10:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].b_lag_count    <= write_stats_matrix[id][idx].b_lag_count   + MAXCNT'('d1); // response awaited, BVALID=1, BID match, but response is not meant for this index -> B-channel lag
                    12'b11?_??_???_10_11:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].b_stall_count  <= write_stats_matrix[id][idx].b_stall_count + MAXCNT'('d1); // B-channel stall
                    12'b11?_??_???_11_11:   write_stats_matrix[id][idx-integer'(wr_recording_done[id])].b_end_count    <= write_stats_matrix[id][idx].b_end_count   + MAXCNT'('d1); // B-channel end of burst
                    default:                wr_no_increment[id][idx] <= 1'b1;
                endcase
            end
        end

        temp_wr_no_increment <= 'b0;
        for (int idx=0; idx<temp_write_stats_last+1; idx++) begin
            if (idx == 0 && merge_cycle) begin     // if merge is happening for temp_stats_arr[0], don't update its counters here, since it is already done in the write_burst_merge_stats function
                continue;
            end
            casez({temp_write_stats_arr[idx].wr_w_outstanding, temp_write_stats_arr[idx].wr_in_progress,
                    // axi_bus.aw_valid,
                    axi_bus.w_valid, axi_bus.w_ready})
                // due to priorization this case needs to be before beat_count
                4'b0?_11:   temp_write_stats_arr[idx-integer'(merge_cycle)].early_beat_count     <= temp_write_stats_arr[idx].early_beat_count    + MAXCNT'('d1); // Early write beat (Before AWV)
                // Throughput measures
                4'b01_10:   temp_write_stats_arr[idx-integer'(merge_cycle)].stall_count          <= temp_write_stats_arr[idx].stall_count + MAXCNT'('d1); // Stall #2
                // Lag measures
                4'b10_??,                                                                                                                                       // Write address lag #1
                4'b01_0?:   temp_write_stats_arr[idx-integer'(merge_cycle)].addr_lag_count       <= temp_write_stats_arr[idx].addr_lag_count      + MAXCNT'('d1); // Write address lag #2
                4'b00_10:   temp_write_stats_arr[idx-integer'(merge_cycle)].early_stall_count    <= temp_write_stats_arr[idx].early_stall_count   + MAXCNT'('d1); // Early Data Stall
                default:    temp_wr_no_increment[idx] <= 1'b1;
            endcase
        end
    end
end

// Write AW outstanding
always @(posedge clk or negedge nres) begin
    if(!nres || clear_measurement) begin
        wr_aw_outstanding       <= 0;
        wr_aw_zero_outstanding  <= 1'b1;
        wr_aw_err               <= 1'b0;
    end else case ({axi_bus.aw_valid && axi_bus.aw_ready,
                    axi_bus.b_valid && axi_bus.b_ready})
        2'b10: begin
            {wr_aw_err, wr_aw_outstanding} <= wr_aw_outstanding + MAXWBURSTS_WIDTH'('d1);
            wr_aw_zero_outstanding <= 1'b0;
        end
        2'b01: begin
            {wr_aw_err, wr_aw_outstanding} <= wr_aw_outstanding - MAXWBURSTS_WIDTH'('d1);
            wr_aw_zero_outstanding <= (wr_aw_outstanding <= 1);
        end
        default: begin end
    endcase
end

// Write W outstanding
always @(posedge clk or negedge nres) begin
    if(!nres || clear_measurement) begin
        wr_w_outstanding        <= 0;
        wr_w_zero_outstanding   <= 1'b1;
        wr_w_err                <= 1'b0;
    end else case ({axi_bus.w_valid && axi_bus.w_ready && axi_bus.w_last,
                    axi_bus.b_valid && axi_bus.b_ready})
        2'b10: begin
            {wr_w_err, wr_w_outstanding} <= wr_w_outstanding + MAXWBURSTS_WIDTH'('d1);
            wr_w_zero_outstanding <= 1'b0;
        end
        2'b01: begin
            {wr_w_err, wr_w_outstanding} <= wr_w_outstanding - MAXWBURSTS_WIDTH'('d1);
            wr_w_zero_outstanding <= (wr_w_outstanding <= 1);
        end
        default: begin end
    endcase
end

// max write outstanding bursts
always_ff @( posedge clk or negedge nres ) begin
    if (!nres || clear_measurement) begin
        wr_w_max_outstanding <= 0;
        wr_aw_max_outstanding <= 0;
    end else if (wr_rec_triggered) begin
        if (wr_w_outstanding + (wr_in_progress ? 1 : 0) > wr_w_max_outstanding) begin
            wr_w_max_outstanding <= wr_w_outstanding + (wr_in_progress ? 1 : 0);
        end

        if (wr_aw_outstanding > wr_aw_max_outstanding) begin
            wr_aw_max_outstanding <= wr_aw_outstanding;
        end
    end
end

//WVALID occured flag - indicates that there was a WVALID on the bus since the last WVALID && WREADY && WLAST
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        wvalid_occured <= 1'b0;
    end else if (axi_bus.w_valid) begin
        if (axi_bus.w_ready && axi_bus.w_last) begin
            wvalid_occured <= 1'b0;
        end else begin
            wvalid_occured <= 1'b1;
        end
    end
end

// Write in progress flag - high bewteen first WVALID until WVALID && WREADY && WLAST
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        wr_in_progress <= 1'b0;
    end else begin
        if (first_wvalid) begin
            if(axi_bus.w_ready && axi_bus.w_last) begin
                wr_in_progress <= 1'b0;
            end else begin
                wr_in_progress <= 1'b1;
            end
        end else begin
            if (axi_bus.w_ready && axi_bus.w_last) begin
                wr_in_progress <= 1'b0;
            end else begin
                wr_in_progress <= wr_in_progress;
            end
        end
    end
end
// Cumulation of total write counter values
always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        wr_total_awvalid_count      <= 0;
        wr_total_beat_count         <= 0;
        wr_total_burst_count        <= 0;
        wr_total_bvalid_count       <= 0;
        wr_total_wvalid_count       <= 0;
        wr_total_idle_count         <= 0;
        wr_total_bus_active         <= 0;
        wr_total_addr_stall_count   <= 0;
        wr_total_addr_lag_count     <= 0;
        wr_total_early_stall_count  <= 0;
        wr_total_b_lag_count        <= 0;
        wr_total_b_stall_count      <= 0;
        wr_total_b_end_count        <= 0;
        wr_total_data_lag_count     <= 0;
        wr_total_stall_count        <= 0;
        wr_total_slow_data_count    <= 0;
        wr_total_awr_early_count    <= 0;
        wr_total_early_beat_count   <= 0;
    end
    if (wr_rec_triggered || (axi_bus.ar_valid && wr_first_in_rec && (start_rcvd || start_measurement))) begin
        if (axi_bus.w_valid && axi_bus.w_ready) begin
            wr_total_beat_count     <= wr_total_beat_count + MAXCNT'('d1);
        end
        if (axi_bus.aw_valid && axi_bus.aw_ready) begin
            wr_total_burst_count    <= wr_total_burst_count + MAXCNT'('d1);
        end
        if (axi_bus.b_valid && axi_bus.b_ready) begin
            wr_total_b_end_count    <= wr_total_b_end_count + MAXCNT'('d1);
        end
        if (axi_bus.aw_valid) begin
            wr_total_awvalid_count  <= wr_total_awvalid_count + MAXCNT'('d1);
        end
        if (axi_bus.w_valid) begin
            wr_total_wvalid_count   <= wr_total_wvalid_count + MAXCNT'('d1);
        end
        if (axi_bus.b_valid) begin
            wr_total_bvalid_count   <= wr_total_bvalid_count + MAXCNT'('d1);
        end
        if (wr_channel_idle) begin
            wr_total_idle_count     <= wr_total_idle_count + MAXCNT'('d1);   // Write idle
        end else begin
            wr_total_bus_active <= wr_total_bus_active + MAXCNT'('d1);
        end
        wr_total_addr_stall_count   <= wr_total_addr_stall_count    + r_wr_total_addr_stall_count;
        wr_total_addr_lag_count     <= wr_total_addr_lag_count      + r_wr_total_addr_lag_count;
        wr_total_early_stall_count  <= wr_total_early_stall_count   + r_wr_total_early_stall_count;
        wr_total_b_lag_count        <= wr_total_b_lag_count         + r_wr_total_b_lag_count;
        wr_total_b_stall_count      <= wr_total_b_stall_count       + r_wr_total_b_stall_count;
        wr_total_data_lag_count     <= wr_total_data_lag_count      + r_wr_total_data_lag_count;
        wr_total_stall_count        <= wr_total_stall_count         + r_wr_total_stall_count;
        wr_total_slow_data_count    <= wr_total_slow_data_count     + r_wr_total_slow_data_count;
        wr_total_awr_early_count    <= wr_total_awr_early_count     + r_wr_total_awr_early_count;
        wr_total_early_beat_count   <= wr_total_early_beat_count    + r_wr_total_early_beat_count;
    end
end

//------------------------------------------
//  Report generation
//------------------------------------------
int fd0, fd1, fd2, fd3, i;
logic [MAXCNT-1:0] snap_active_count;
logic [MAXCNT-1:0] read_latencies[], read_init_delays[], read_durations[];
logic [MAXCNT-1:0] snap_rd_total_idle_count, snap_rd_total_beat_count, snap_rd_total_burst_count,
                snap_rd_total_ar_stall_count, snap_rd_total_r_stall_count, snap_rd_total_arvalid_count,
                snap_rd_total_rvalid_count, snap_rd_total_bus_active_count, snap_rd_total_slow_link_count,
                snap_rd_total_ar_cycle_count, snap_rd_total_lag_count, snap_rd_total_lag_cycles,
                snap_rd_total_throughput_dnm_count;

logic [MAXCNT-1:0]  max_read_latency, min_read_latency, median_read_latency,
                    max_read_latency_idx, min_read_latency_idx, max_read_latency_time,
                    min_read_latency_time;
logic [MAXCNT-1:0]  max_read_init_delay, min_read_init_delay, median_read_init_delay,
                    max_read_init_delay_idx, min_read_init_delay_idx, max_read_init_delay_time,
                    min_read_init_delay_time;
logic [MAXCNT-1:0]  max_read_duration, min_read_duration, median_read_duration,
                    max_read_duration_idx, min_read_duration_idx, max_read_duration_time,
                    min_read_duration_time;
real avg_read_latency, avg_read_init_delay, avg_read_duration;

`ifndef THROUGHPUT_OFF
    real read_throughput[];
    logic [MAXCNT-1:0]  max_read_throughput_idx, min_read_throughput_idx, max_read_throughput_time,
                        min_read_throughput_time;
    real avg_read_throughput, max_read_throughput, min_read_throughput, median_read_throughput;
`endif

always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        snap_active_count <= MAXCNT'('d0);
    end else if ((wr_rec_triggered && axi_bus.b_ready && axi_bus.b_valid) || (rd_rec_triggered && axi_bus.r_ready && axi_bus.r_valid && axi_bus.r_last)) begin
        snap_active_count <= active_count + 1;  // +1 because current cycle counts as active cycle since there was a burst that ended, but active_count increment happens in the next cycle. So if this is the last burst, the last cycle will also be counted in the snapshot
    end
end

always@(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        // initialization of read variables
        max_read_latency        = MAXCNT'('d0);
        max_read_latency_time   = MAXCNT'('d0);
        max_read_latency_idx    = MAXCNT'('d0);
        min_read_latency        = {MAXCNT{1'b1}};
        min_read_latency_time   = MAXCNT'('d0);
        min_read_latency_idx    = MAXCNT'('d0);
        median_read_latency     = MAXCNT'('d0);
        avg_read_latency        = 0.0;
        read_latencies          = new[0];

        max_read_init_delay         = MAXCNT'('d0);
        max_read_init_delay_time    = MAXCNT'('d0);
        max_read_init_delay_idx     = MAXCNT'('d0);
        min_read_init_delay         = {MAXCNT{1'b1}};
        min_read_init_delay_time    = MAXCNT'('d0);
        min_read_init_delay_idx     = MAXCNT'('d0);
        median_read_init_delay      = MAXCNT'('d0);
        avg_read_init_delay         = 0.0;
        read_init_delays            = new[0];

        max_read_duration       = MAXCNT'('d0);
        max_read_duration_time  = MAXCNT'('d0);
        max_read_duration_idx   = MAXCNT'('d0);
        min_read_duration       = {MAXCNT{1'b1}};
        min_read_duration_time  = MAXCNT'('d0);
        min_read_duration_idx   = MAXCNT'('d0);
        median_read_duration    = MAXCNT'('d0);
        avg_read_duration       = 0.0;
        read_durations          = new[0];

        `ifndef THROUGHPUT_OFF
            max_read_throughput             = 0.0;
            max_read_throughput_time        = MAXCNT'('d0);
            max_read_throughput_idx         = MAXCNT'('d0);
            min_read_throughput             = 2.0;  // equates to 200% which is greater than any achievable throughput
            min_read_throughput_time        = MAXCNT'('d0);
            min_read_throughput_idx         = MAXCNT'('d0);
            rd_total_throughput_dnm_count   = MAXCNT'('d0);
            median_read_throughput          = 0.0;
            avg_read_throughput             = 0.0;
            read_throughput                 = new[0];
        `endif

        snap_rd_total_idle_count            = MAXCNT'('d0);
        snap_rd_total_beat_count            = MAXCNT'('d0);
        snap_rd_total_burst_count           = MAXCNT'('d0);
        snap_rd_total_ar_stall_count        = MAXCNT'('d0);
        snap_rd_total_r_stall_count         = MAXCNT'('d0);
        snap_rd_total_rvalid_count          = MAXCNT'('d0);
        snap_rd_total_bus_active_count      = MAXCNT'('d0);
        snap_rd_total_slow_link_count       = MAXCNT'('d0);
        snap_rd_total_ar_cycle_count        = MAXCNT'('d0);
        snap_rd_total_lag_count             = MAXCNT'('d0);
        snap_rd_total_lag_cycles            = MAXCNT'('d0);
        snap_rd_total_throughput_dnm_count  = MAXCNT'('d0);
        snap_rd_total_arvalid_count         = MAXCNT'('d0);
    end else if (rd_recording_saved) begin  // create snapshot, everytime burst is saved to results array
        read_latencies      = new[read_stats_last+1];
        read_init_delays    = new[read_stats_last+1];
        read_durations      = new[read_stats_last+1];
        read_throughput     = new[read_stats_last+1];
        avg_read_throughput     = 0.0;
        rd_total_throughput_dnm_count   = MAXCNT'('d0);
        // calculate read max and min values, preparatory calculations for averages
        for(i=0;i<(read_stats_last+1);i++) begin

            read_latencies[i] = read_stats_arr[i].latency;
            if (read_latencies[i] > max_read_latency) begin
                max_read_latency        = read_latencies[i];
                max_read_latency_time   = read_stats_arr[i].time_issued;
                max_read_latency_idx    = i;
            end
            if (read_latencies[i] < min_read_latency) begin
                min_read_latency        = read_latencies[i];
                min_read_latency_time   = read_stats_arr[i].time_issued;
                min_read_latency_idx    = i;
            end

            read_init_delays[i] = read_stats_arr[i].init_delay;
            if (read_init_delays[i] > max_read_init_delay) begin
                max_read_init_delay        = read_init_delays[i];
                max_read_init_delay_time   = read_stats_arr[i].time_issued;
                max_read_init_delay_idx    = i;
            end
            if (read_init_delays[i] < min_read_init_delay) begin
                min_read_init_delay        = read_init_delays[i];
                min_read_init_delay_time   = read_stats_arr[i].time_issued;
                min_read_init_delay_idx    = i;
            end

            read_durations[i] = read_stats_arr[i].duration;
            if (read_durations[i] > max_read_duration) begin
                max_read_duration        = read_durations[i];
                max_read_duration_time   = read_stats_arr[i].time_issued;
                max_read_duration_idx    = i;
            end
            if (read_durations[i] < min_read_duration) begin
                min_read_duration        = read_durations[i];
                min_read_duration_time   = read_stats_arr[i].time_issued;
                min_read_duration_idx    = i;
            end

            `ifndef THROUGHPUT_OFF
                if(read_stats_arr[i].throughput_dnm != 0) begin
                    read_throughput[i] = $itor(read_stats_arr[i].beat_count)/$itor(read_stats_arr[i].throughput_dnm);
                    avg_read_throughput    += read_throughput[i];
                end else begin
                    $warning("division by zero averted during read throughput calculation");
                    read_throughput[i] = 0.0;
                end
                rd_total_throughput_dnm_count += read_stats_arr[i].throughput_dnm;
                if (read_throughput[i] > max_read_throughput) begin
                    max_read_throughput        = read_throughput[i];
                    max_read_throughput_time   = read_stats_arr[i].time_issued;
                    max_read_throughput_idx    = i;
                end
                if (read_throughput[i] < min_read_throughput) begin
                    min_read_throughput        = read_throughput[i];
                    min_read_throughput_time   = read_stats_arr[i].time_issued;
                    min_read_throughput_idx    = i;
                end
            `endif
        end

        // if min values still at default, set min values to 0
        if (min_read_latency == {MAXCNT{1'b1}}) begin
            min_read_latency = MAXCNT'('d0);
        end
        if (min_read_duration == {MAXCNT{1'b1}}) begin
            min_read_duration = MAXCNT'('d0);
        end
        if (min_read_init_delay == {MAXCNT{1'b1}}) begin
            min_read_init_delay = MAXCNT'('d0);
        end
        if (min_read_throughput == 2.0) begin
            min_read_throughput = 0.0;
        end

        // calculate read average values
        //$display("rd_total_burst_count: %d", rd_total_burst_count);
        if(rd_total_burst_count != 0) begin
            avg_read_latency    = $itor(read_latencies.sum())/$itor(rd_total_burst_count);
            avg_read_init_delay = $itor(read_init_delays.sum())/$itor(rd_total_burst_count);
            avg_read_duration   = $itor(read_durations.sum())/$itor(rd_total_burst_count);
            `ifndef THROUGHPUT_OFF
                avg_read_throughput = avg_read_throughput/$itor(rd_total_burst_count);
            `endif
        end else begin
            $warning("No Read Bursts recorded, division by zero averted during avg calculation");
            avg_read_latency    = 0.0;
            avg_read_init_delay = 0.0;
            avg_read_duration   = 0.0;
            `ifndef THROUGHPUT_OFF
                avg_read_throughput = 0.0;
            `endif
        end

        // calculate read median values
        read_latencies.sort();
        read_init_delays.sort();
        read_durations.sort();
        median_read_latency     = read_latencies[read_stats_last/2];
        median_read_init_delay  = read_init_delays[read_stats_last/2];
        median_read_duration    = read_durations[read_stats_last/2];
        `ifndef THROUGHPUT_OFF
            read_throughput.sort();
            median_read_throughput  = read_throughput[read_stats_last/2];
        `endif

        // save total counter values in snapshot variables
        snap_rd_total_idle_count            = rd_total_idle_count;
        snap_rd_total_beat_count            = rd_total_beat_count;
        snap_rd_total_burst_count           = rd_total_burst_count;
        snap_rd_total_ar_stall_count        = rd_total_ar_stall_count;
        snap_rd_total_r_stall_count         = rd_total_r_stall_count;
        snap_rd_total_rvalid_count          = rd_total_rvalid_count;
        snap_rd_total_bus_active_count      = rd_total_bus_active_count;
        snap_rd_total_slow_link_count       = rd_total_slow_link_count;
        snap_rd_total_ar_cycle_count        = rd_total_ar_cycle_count;
        snap_rd_total_lag_count             = rd_total_lag_count;
        snap_rd_total_lag_cycles            = rd_total_lag_cycles;
        snap_rd_total_throughput_dnm_count  = rd_total_throughput_dnm_count;
        snap_rd_total_arvalid_count         = rd_total_arvalid_count;
    end
end

logic [MAXCNT-1:0] write_latencies[], write_init_delays[], write_durations[], write_backpressure[];
logic [MAXCNT-1:0] snap_wr_total_awvalid_count, snap_wr_total_beat_count, snap_wr_total_burst_count,
                    snap_wr_total_bvalid_count, snap_wr_total_wvalid_count, snap_wr_total_idle_count,
                    snap_wr_total_bus_active, snap_wr_total_addr_stall_count, snap_wr_total_addr_lag_count,
                    snap_wr_total_early_stall_count, snap_wr_total_b_lag_count, snap_wr_total_b_stall_count,
                    snap_wr_total_b_end_count, snap_wr_total_data_lag_count, snap_wr_total_stall_count,
                    snap_wr_total_slow_data_count, snap_wr_total_awr_early_count, snap_wr_total_early_beat_count,
                    snap_wr_total_throughput_dnm_count;
`ifndef THROUGHPUT_OFF
    real write_throughput[];
`endif
logic [MAXCNT-1:0]  max_write_latency, min_write_latency, median_write_latency,
                    max_write_latency_idx, min_write_latency_idx, max_write_latency_time,
                    min_write_latency_time;
logic [MAXCNT-1:0]  max_write_backpressure, min_write_backpressure, median_write_backpressure,
                    max_write_backpressure_idx, min_write_backpressure_idx, max_write_backpressure_time,
                    min_write_backpressure_time;
logic [MAXCNT-1:0]  max_write_init_delay, min_write_init_delay, median_write_init_delay,
                    max_write_init_delay_idx, min_write_init_delay_idx, max_write_init_delay_time,
                    min_write_init_delay_time;
logic [MAXCNT-1:0]  max_write_duration, min_write_duration, median_write_duration,
                    max_write_duration_idx, min_write_duration_idx, max_write_duration_time,
                    min_write_duration_time;
`ifndef THROUGHPUT_OFF
    logic [MAXCNT-1:0]  max_write_throughput_idx, min_write_throughput_idx, max_write_throughput_time,
                        min_write_throughput_time;
    real avg_write_throughput, max_write_throughput, min_write_throughput, median_write_throughput;
`endif
real avg_write_latency, avg_write_init_delay, avg_write_duration, avg_write_backpressure;

always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        // intialization of write variables
        max_write_latency       = MAXCNT'('d0);
        max_write_latency_time  = MAXCNT'('d0);
        max_write_latency_idx   = MAXCNT'('d0);
        min_write_latency       = {MAXCNT{1'b1}};
        min_write_latency_time  = MAXCNT'('d0);
        min_write_latency_idx   = MAXCNT'('d0);
        median_write_latency    = MAXCNT'('d0);
        avg_write_latency       = 0.0;
        write_latencies         = new[0];

        max_write_init_delay        = MAXCNT'('d0);
        max_write_init_delay_time   = MAXCNT'('d0);
        max_write_init_delay_idx    = MAXCNT'('d0);
        min_write_init_delay        = {MAXCNT{1'b1}};
        min_write_init_delay_time   = MAXCNT'('d0);
        min_write_init_delay_idx    = MAXCNT'('d0);
        median_write_init_delay     = MAXCNT'('d0);
        avg_write_init_delay        = 0.0;
        write_init_delays           = new[0];

        max_write_duration      = MAXCNT'('d0);
        max_write_duration_time = MAXCNT'('d0);
        max_write_duration_idx  = MAXCNT'('d0);
        min_write_duration      = {MAXCNT{1'b1}};
        min_write_duration_time = MAXCNT'('d0);
        min_write_duration_idx  = MAXCNT'('d0);
        median_write_duration   = MAXCNT'('d0);
        avg_write_duration      = 0.0;
        write_durations         = new[0];

        max_write_backpressure      = MAXCNT'('d0);
        max_write_backpressure_time = MAXCNT'('d0);
        max_write_backpressure_idx  = MAXCNT'('d0);
        min_write_backpressure      = {MAXCNT{1'b1}};
        min_write_backpressure_time = MAXCNT'('d0);
        min_write_backpressure_idx  = MAXCNT'('d0);
        median_write_backpressure   = MAXCNT'('d0);
        avg_write_backpressure      = 0.0;
        write_backpressure          = new[0];

        `ifndef THROUGHPUT_OFF
            max_write_throughput            = 0.0;
            max_write_throughput_time       = MAXCNT'('d0);
            max_write_throughput_idx        = MAXCNT'('d0);
            min_write_throughput            = 2.0;
            min_write_throughput_time       = MAXCNT'('d0);
            min_write_throughput_idx        = MAXCNT'('d0);
            wr_total_throughput_dnm_count   = MAXCNT'('d0);
            median_write_throughput         = 0.0;
            avg_write_throughput            = 0.0;
            write_throughput                = new[0];
        `endif

        snap_wr_total_awvalid_count         = MAXCNT'('d0);
        snap_wr_total_beat_count            = MAXCNT'('d0);
        snap_wr_total_burst_count         = MAXCNT'('d0);
        snap_wr_total_bvalid_count          = MAXCNT'('d0);
        snap_wr_total_wvalid_count          = MAXCNT'('d0);
        snap_wr_total_idle_count            = MAXCNT'('d0);
        snap_wr_total_bus_active            = MAXCNT'('d0);
        snap_wr_total_addr_stall_count      = MAXCNT'('d0);
        snap_wr_total_addr_lag_count        = MAXCNT'('d0);
        snap_wr_total_early_stall_count     = MAXCNT'('d0);
        snap_wr_total_b_lag_count           = MAXCNT'('d0);
        snap_wr_total_b_stall_count         = MAXCNT'('d0);
        snap_wr_total_b_end_count           = MAXCNT'('d0);
        snap_wr_total_data_lag_count        = MAXCNT'('d0);
        snap_wr_total_stall_count           = MAXCNT'('d0);
        snap_wr_total_slow_data_count       = MAXCNT'('d0);
        snap_wr_total_awr_early_count       = MAXCNT'('d0);
        snap_wr_total_early_beat_count      = MAXCNT'('d0);
        snap_wr_total_throughput_dnm_count  = MAXCNT'('d0);
    end else if (wr_recording_saved) begin
        write_latencies     = new[write_stats_last+1];
        write_init_delays   = new[write_stats_last+1];
        write_durations     = new[write_stats_last+1];
        write_backpressure  = new[write_stats_last+1];
        write_throughput    = new[write_stats_last+1];
        avg_write_throughput    = 0.0;
        wr_total_throughput_dnm_count   = MAXCNT'('d0);
        // calculate write max and min values, preparatory calculations for averages
        for(i=0;i<(write_stats_last+1);i++) begin
            write_latencies[i]      = write_stats_arr[i].latency;
            if (write_latencies[i] > max_write_latency) begin
                max_write_latency       = write_latencies[i];
                max_write_latency_time  = write_stats_arr[i].time_issued;
                max_write_latency_idx   = i;
            end
            if (write_latencies[i] < min_write_latency) begin
                min_write_latency       = write_latencies[i];
                min_write_latency_time  = write_stats_arr[i].time_issued;
                min_write_latency_idx   = i;
            end

            write_init_delays[i]      = write_stats_arr[i].init_delay;
            if (write_init_delays[i] > max_write_init_delay) begin
                max_write_init_delay       = write_init_delays[i];
                max_write_init_delay_time  = write_stats_arr[i].time_issued;
                max_write_init_delay_idx   = i;
            end
            if (write_init_delays[i] < min_write_init_delay) begin
                min_write_init_delay       = write_init_delays[i];
                min_write_init_delay_time  = write_stats_arr[i].time_issued;
                min_write_init_delay_idx   = i;
            end

            write_durations[i]      = write_stats_arr[i].duration;
            if (write_durations[i] > max_write_duration) begin
                max_write_duration       = write_durations[i];
                max_write_duration_time  = write_stats_arr[i].time_issued;
                max_write_duration_idx   = i;
            end
            if (write_durations[i] < min_write_duration) begin
                min_write_duration       = write_durations[i];
                min_write_duration_time  = write_stats_arr[i].time_issued;
                min_write_duration_idx   = i;
            end

            write_backpressure[i]   = write_stats_arr[i].backpressure;
            if (write_backpressure[i] > max_write_backpressure) begin
                max_write_backpressure      = write_backpressure[i];
                max_write_backpressure_time = write_stats_arr[i].time_issued;
                max_write_backpressure_idx  = i;
            end
            if (write_backpressure[i] < min_write_backpressure) begin
                min_write_backpressure      = write_backpressure[i];
                min_write_backpressure_time = write_stats_arr[i].time_issued;
                min_write_backpressure      = i;
            end

            `ifndef THROUGHPUT_OFF
                if(write_stats_arr[i].throughput_dnm != 0) begin
                    write_throughput[i] = $itor(write_stats_arr[i].beat_count + write_stats_arr[i].early_beat_count)/$itor(write_stats_arr[i].throughput_dnm);
                    avg_write_throughput += write_throughput[i];
                end else begin
                    $warning("division by zero averted during write throughput calculation");
                    write_throughput[i] = 0.0;
                end
                wr_total_throughput_dnm_count += write_stats_arr[i].throughput_dnm;
                if (write_throughput[i] > max_write_throughput) begin
                    max_write_throughput        = write_throughput[i];
                    max_write_throughput_time   = write_stats_arr[i].time_issued;
                    max_write_throughput_idx    = i;
                end
                if (write_throughput[i] < min_write_throughput) begin
                    min_write_throughput        = write_throughput[i];
                    min_write_throughput_time   = write_stats_arr[i].time_issued;
                    min_write_throughput_idx    = i;
                end
            `endif
        end

        if (min_write_latency == {MAXCNT{1'b1}}) begin
            min_write_latency = MAXCNT'('d0);
        end
        if (min_write_duration == {MAXCNT{1'b1}}) begin
            min_write_duration = MAXCNT'('d0);
        end
        if (min_write_init_delay == {MAXCNT{1'b1}}) begin
            min_write_init_delay = MAXCNT'('d0);
        end
        if (min_write_backpressure == {MAXCNT{1'b1}}) begin
            min_write_backpressure = MAXCNT'('d0);
        end
        if (min_write_throughput == 2.0) begin
            min_write_throughput = 0.0;
        end

        // calculate write average values
        //$display("wr_total_burst_count: %d", wr_total_burst_count);
        if(wr_total_burst_count != 0) begin
            avg_write_latency       = $itor(write_latencies.sum())/$itor(wr_total_burst_count);
            avg_write_init_delay    = $itor(write_init_delays.sum())/$itor(wr_total_burst_count);
            avg_write_duration      = $itor(write_durations.sum())/$itor(wr_total_burst_count);
            avg_write_backpressure  = $itor(write_backpressure.sum())/$itor(wr_total_burst_count);
            `ifndef THROUGHPUT_OFF
                avg_write_throughput = avg_write_throughput/$itor(wr_total_burst_count);
            `endif
        end else begin
            $warning("No AWBURSTS recorded, division by zero averted during avg calculation");
            avg_write_latency       = 0.0;
            avg_write_init_delay    = 0.0;
            avg_write_duration      = 0.0;
            avg_write_backpressure  = 0.0;
            `ifndef THROUGHPUT_OFF
                avg_write_throughput = 0.0;
            `endif
        end

        // calculate write median values
        write_latencies.sort();
        write_init_delays.sort();
        write_durations.sort();
        write_backpressure.sort();
        median_write_latency        = write_latencies[write_stats_last/2];
        median_write_init_delay     = write_init_delays[write_stats_last/2];
        median_write_duration       = write_durations[write_stats_last/2];
        median_write_backpressure   = write_backpressure[write_stats_last/2];
        `ifndef THROUGHPUT_OFF
            write_throughput.sort();
            median_write_throughput = write_throughput[write_stats_last/2];
        `endif

        // save total write counter values to snapshot
        snap_wr_total_awvalid_count         <= wr_total_awvalid_count;
        snap_wr_total_beat_count            <= wr_total_beat_count;
        snap_wr_total_burst_count           <= wr_total_burst_count;
        snap_wr_total_bvalid_count          <= wr_total_bvalid_count;
        snap_wr_total_wvalid_count          <= wr_total_wvalid_count;
        snap_wr_total_idle_count            <= wr_total_idle_count;
        snap_wr_total_bus_active            <= wr_total_bus_active;
        snap_wr_total_addr_stall_count      <= wr_total_addr_stall_count;
        snap_wr_total_addr_lag_count        <= wr_total_addr_lag_count;
        snap_wr_total_early_stall_count     <= wr_total_early_stall_count;
        snap_wr_total_b_lag_count           <= wr_total_b_lag_count;
        snap_wr_total_b_stall_count         <= wr_total_b_stall_count;
        snap_wr_total_b_end_count           <= wr_total_b_end_count;
        snap_wr_total_data_lag_count        <= wr_total_data_lag_count;
        snap_wr_total_stall_count           <= wr_total_stall_count;
        snap_wr_total_slow_data_count       <= wr_total_slow_data_count;
        snap_wr_total_awr_early_count       <= wr_total_awr_early_count;
        snap_wr_total_early_beat_count      <= wr_total_early_beat_count;
        snap_wr_total_throughput_dnm_count  <= wr_total_throughput_dnm_count;
    end
end

logic file_written;

always @(posedge clk or negedge nres) begin
    if (!nres || clear_measurement) begin
        file_written    <= 1'b0;
    end
    if (print_measurement && !file_written) begin
        // write output to file
        fd0 = $fopen($sformatf("%s.txt", OUT_FILE_NAME));
        fd1 = $fopen($sformatf("%s_read.csv", RAW_OUT_FILE_NAME));
        fd2 = $fopen($sformatf("%s_write.csv", RAW_OUT_FILE_NAME));
        fd3 = $fopen($sformatf("%s_total.csv", RAW_OUT_FILE_NAME));

        if(fd0) begin
            //read transfer legend
            $fdisplay(fd0, "--------------------------------- READ_TRANSFER_LEGEND -----------------------------------");
            $fdisplay(fd0, "Cycle:               | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |14 |15 |");
            $fdisplay(fd0, "CLK      ----->  /‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/");
            $fdisplay(fd0, "ARADDR   ----->  #[   A   ]########################################################");
            $fdisplay(fd0, "ARVALID  ----->  _/‾‾‾‾‾‾‾\\________________________________________________________");
            $fdisplay(fd0, "ARREADY  ----->  _____/‾‾‾\\########################################################");
            $fdisplay(fd0, "RDATA    ----->  #############################[  D0  ]####[  D1  ]####[  D2  ]#####");
            $fdisplay(fd0, "RLAST    ----->  #############################________________________/‾‾‾‾‾‾‾#####");
            $fdisplay(fd0, "RVALID   ----->  _____________________________/‾‾‾‾‾‾‾\\___/‾‾‾‾‾‾‾\\___/‾‾‾‾‾‾‾\\____");
            $fdisplay(fd0, "RREADY   ----->  #############################____/‾‾‾####____/‾‾‾####____/‾‾‾\\____");
            $fdisplay(fd0, "count as:            |RAS|RAC|LAG|LAG|LAG|LAG|LAG|RS |DAT|SRL|RS |DAT|SRL|RS |DAT|IDL|");
            $fdisplay(fd0, "Read Latency:        |<----------------------------->|   |<----->|   |<----->|");
            $fdisplay(fd0, "                      ARVALID bis RVALID && RREADY && RLAST minus Burstlänge (hier 12 Zyklen)");
            $fdisplay(fd0, "Initiation Delay:    |<------------------------->|");
            $fdisplay(fd0, "                      ARVALID bis erstes RVALID (hier 7 Zyklen)");
            $fdisplay(fd0, "Throughput cycles:                               |<----------------------------->|");
            $fdisplay(fd0, "                      RVALID && RREADY bis RVALID && RREADY && RLAST (hier 8 Zyklen)");
            $fdisplay(fd0, "Transfer duration:   |<--------------------------------------------------------->|");
            $fdisplay(fd0, "                      ARVALID bis RVALID && RREADY && RLAST (hier 15 Zyklen)");
            $fdisplay(fd0, "");
            $fdisplay(fd0, "(Readzyklus Kategorisierung:");
            $fdisplay(fd0, "https://zipcpu.com/img/axiperf/rdcategories.svg");
            $fdisplay(fd0, "https://zipcpu.com/img/axiperf/rdburst-annotated.svg)");
            $fdisplay(fd0, "\n");

            if(snap_rd_total_burst_count == 0) begin
                $fdisplay(fd0, "");
                $fdisplay(fd0, "############################### NO READ BURSTS RECORDED ###############################");
                $fdisplay(fd0, "");
            end else begin
                $fdisplay(fd0, "------------------------------- READ_LATENCY_STATISTICS -------------------------------");
                $fdisplay(fd0, "MAX_READ_LATENCY:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                max_read_latency,
                                max_read_latency_time,
                                read_stats_arr[max_read_latency_idx].address,
                                read_stats_arr[max_read_latency_idx].id,
                                max_read_latency_idx);
                $fdisplay(fd0, "MIN_READ_LATENCY:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                min_read_latency,
                                min_read_latency_time,
                                read_stats_arr[min_read_latency_idx].address,
                                read_stats_arr[min_read_latency_idx].id,
                                min_read_latency_idx);
                $fdisplay(fd0, "AVG_READ_LATENCY:\t%10.2f", avg_read_latency);
                $fdisplay(fd0, "MEDIAN_READ_LATENCY:\t%10d", median_read_latency);

                $fdisplay(fd0, "------------------------------- READ_INIT_DELAY_STATISTICS ----------------------------");
                $fdisplay(fd0, "MAX_READ_INIT_DELAY:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                max_read_init_delay,
                                max_read_init_delay_time,
                                read_stats_arr[max_read_init_delay_idx].address,
                                read_stats_arr[max_read_init_delay_idx].id,
                                max_read_init_delay_idx);
                $fdisplay(fd0, "MIN_READ_INIT_DELAY:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                min_read_init_delay,
                                min_read_init_delay_time,
                                read_stats_arr[min_read_init_delay_idx].address,
                                read_stats_arr[min_read_init_delay_idx].id,
                                min_read_init_delay_idx);
                $fdisplay(fd0, "AVG_READ_INIT_DELAY:\t%10.2f", avg_read_init_delay);
                $fdisplay(fd0, "MEDIAN_READ_INIT_DELAY:\t%10d", median_read_init_delay);

                $fdisplay(fd0, "------------------------------- READ_DURATION_STATISTICS ------------------------------");
                $fdisplay(fd0, "MAX_READ_DURATION:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                max_read_duration,
                                max_read_duration_time,
                                read_stats_arr[max_read_duration_idx].address,
                                read_stats_arr[max_read_duration_idx].id,
                                max_read_duration_idx);
                $fdisplay(fd0, "MIN_READ_DURATION:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                min_read_duration,
                                min_read_duration_time,
                                read_stats_arr[min_read_duration_idx].address,
                                read_stats_arr[max_read_duration_idx].id,
                                min_read_duration_idx);
                $fdisplay(fd0, "AVG_READ_DURATION:\t%10.2f", avg_read_duration);
                $fdisplay(fd0, "MEDIAN_READ_DURATION:\t%10d", median_read_duration);

                `ifndef THROUGHPUT_OFF
                    $fdisplay(fd0, "------------------------------- READ_THROUGHPUT_STATISTICS ----------------------------");
                    $fdisplay(fd0, "MAX_READ_THROUGHPUT:\t\t%5.2f %%, %10d beats / %10d cycles issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                    max_read_throughput*100.0,
                                    read_stats_arr[max_read_throughput_idx].beat_count,
                                    read_stats_arr[max_read_throughput_idx].throughput_dnm,
                                    max_read_throughput_time,
                                    read_stats_arr[max_read_throughput_idx].address,
                                    read_stats_arr[max_read_throughput_idx].id,
                                    max_read_throughput_idx);
                    $fdisplay(fd0, "MIN_READ_THROUGHPUT:\t\t%5.2f %%, %10d beats / %10d cycles issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                    min_read_throughput*100.0,
                                    read_stats_arr[min_read_throughput_idx].beat_count,
                                    read_stats_arr[min_read_throughput_idx].throughput_dnm,
                                    min_read_throughput_time,
                                    read_stats_arr[min_read_throughput_idx].address,
                                    read_stats_arr[min_read_throughput_idx].id,
                                    min_read_throughput_idx);
                    $fdisplay(fd0, "AVG_READ_THROUGHPUT_PER_BURST:\t\t%5.2f %%", avg_read_throughput*100.0);
                    // $fdisplay(fd0, "AVG_READ_THROUGHPUT:\t\t%5.2f %%, %10d beats / %10d cycles", ($itor(snap_rd_total_burst_count)/$itor(snap_rd_total_throughput_dnm_count))*100,
                    //                                                                             snap_rd_total_beat_count,
                    //                                                                             snap_rd_total_throughput_dnm_count);
                    $fdisplay(fd0, "MEDIAN_READ_THROUGHPUT:\t\t%5.2f %%", median_read_throughput*100.0);
                `endif

                $fdisplay(fd0, "------------------------------- TOTAL_READ_STATISTICS ---------------------------------");
                $fdisplay(fd0, "TOTAL_READ_BURSTS:\t\t%10d", snap_rd_total_burst_count);
                $fdisplay(fd0, "TOTAL_READ_BEATS:\t\t%10d", snap_rd_total_beat_count);
                $fdisplay(fd0, "TOTAL_READ_BUS_ACTIVE_CYCLES:\t%10d", snap_rd_total_bus_active_count);
                $fdisplay(fd0, "TOTAL_READ_BUS_IDLE_CYCLES:\t%10d", snap_rd_total_idle_count);
            end


            // write transfer legend
            $fdisplay(fd0, "\n\n");
            if (temp_stats_created) begin
                $fdisplay(fd0, "------------------------------------------------------------------------------------------");
                $fdisplay(fd0, "#   first WVALID preceded first AWVALID of burst, PerfMonitor behaviour is implemented   #");
                $fdisplay(fd0, "#   but largely untested due to lack of stimuli                                          #");
                $fdisplay(fd0, "------------------------------------------------------------------------------------------");
            end
            $fdisplay(fd0, "--------------------------------- WRITE_TRANSFER_LEGEND -------------------------------------------------------------");
            $fdisplay(fd0, "Cycle:              | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |14 |15 |16 | ~ |17 |18 |19 |20 |21 |22 |");
            $fdisplay(fd0, "CLK      -----> /‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/ ~ /‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾\\_/‾");
            $fdisplay(fd0, "AWADDR   -----> #[   A   ]############################################################### ~ #############[ A ]########");
            $fdisplay(fd0, "AWVALID  -----> _/‾‾‾‾‾‾‾\\_______________________________________________________________ ~ _____________/‾‾‾\\________");
            $fdisplay(fd0, "AWREADY  -----> _____/‾‾‾\\_______________________________________________________________ ~ _____________/‾‾‾\\________");
            $fdisplay(fd0, "WDATA    -----> #####################[  D0  ][  D1  ]####[D2][D3]######################## ~ #[  D0   ]################");
            $fdisplay(fd0, "WLAST    -----> #####################________________________/‾‾‾\\####################### ~ #____/‾‾‾\\________________");
            $fdisplay(fd0, "WVALID   -----> _____________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\\___/‾‾‾‾‾‾‾\\_______________________ ~ _/‾‾‾‾‾‾‾\\________________");
            $fdisplay(fd0, "WREADY   -----> #________________________/‾‾‾\\___/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\\####################### ~ ____/‾‾‾‾\\________________");
            $fdisplay(fd0, "BRESP    -----> #############################################################[OK ]####### ~ #################[OK ]###");
            $fdisplay(fd0, "BVALID   -----> _____________________________________________________/‾‾‾‾‾‾‾‾‾‾‾\\_______ ~ _________________/‾‾‾\\____");
            $fdisplay(fd0, "BREADY   -----> _____________________________________________________________/‾‾‾\\_______ ~ _________________/‾‾‾\\____");
            $fdisplay(fd0, "count as:           |AS |AW |LAG|LAG|LAG|WS1|DAT|WS1|DAT|SWD|DAT|DAT|BLG|BST|BST|EOB|IDL| ~ |IDL|EDS|EWB|LAG|WAL|EOB|");
            $fdisplay(fd0, "Write Latency:      |<--------------------->|   |<->|   |<->|       |<------------->|     ~     |<->|   |<--------->|");
            $fdisplay(fd0, "                     AWVALID bis BAVLID && BREADY minus AWLEN (hier 12 Zyklen)            ~ WVALID bis BVALID && BREADY minus AWLEN");
            $fdisplay(fd0, "Initiation Delay:   |<----------------->|                                                 ~     |<-------------->|");
            $fdisplay(fd0, "                     AWVALID bis erstes WVALID, hier 5 Zyklen                             ~ WVALID bis erstes AWVALID");
            $fdisplay(fd0, "Throughput cycles:                      |<------------------------->|                     ~     |<----->|");
            $fdisplay(fd0, "                     WVALID bis BVALID && BREADY (hier 7 Zyklen)                          ~ erstes WVALID bis WVALID && WREADY && WLAST");
            $fdisplay(fd0, "Transfer duration:  |<------------------------------------------------------------->|     ~     |<----------------->|");
            $fdisplay(fd0, "                     AWVALID bis BVALID && BREADY (hier 16 Zyklen)                        ~ WVALID bis BVALID && BREADY");
            $fdisplay(fd0, "");
            $fdisplay(fd0, "(Writezyklus Kategorisierung:");
            $fdisplay(fd0, "https://zipcpu.com/img/axiperf/wrcategories.svg");
            $fdisplay(fd0, "https://zipcpu.com/img/axiperf/wrburst-annotated.svg)");
            $fdisplay(fd0, "\n");

            if (snap_wr_total_burst_count == 0) begin
                $fdisplay(fd0, "");
                $fdisplay(fd0, "############################### NO WRITE BURSTS RECORDED ##############################");
                $fdisplay(fd0, "");
            end else begin
                $fdisplay(fd0, "------------------------------- WRITE_LATENCY_STATISTICS ------------------------------");
                $fdisplay(fd0, "MAX_WRITE_LATENCY:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                max_write_latency,
                                max_write_latency_time,
                                write_stats_arr[max_write_latency_idx].address,
                                write_stats_arr[max_write_latency_idx].id,
                                max_write_latency_idx);
                $fdisplay(fd0, "MIN_WRITE_LATENCY:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                min_write_latency,
                                min_write_latency_time,
                                write_stats_arr[min_write_latency_idx].address,
                                write_stats_arr[min_write_latency_idx].id,
                                min_write_latency_idx);
                $fdisplay(fd0, "AVG_WRITE_LATENCY:\t%10.2f", avg_write_latency);
                $fdisplay(fd0, "MEDIAN_WRITE_LATENCY:\t%10d", median_write_latency);

                $fdisplay(fd0, "------------------------------- WRITE_INIT_DELAY_STATISTICS ---------------------------");
                $fdisplay(fd0, "MAX_WRITE_INIT_DELAY:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                max_write_init_delay,
                                max_write_init_delay_time,
                                write_stats_arr[max_write_init_delay_idx].address,
                                write_stats_arr[max_write_init_delay_idx].id,
                                max_write_init_delay_idx);
                $fdisplay(fd0, "MIN_WRITE_INIT_DELAY:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                min_write_init_delay,
                                min_write_init_delay_time,
                                write_stats_arr[min_write_init_delay_idx].address,
                                write_stats_arr[min_write_init_delay_idx].id,
                                min_write_init_delay_idx);
                $fdisplay(fd0, "AVG_WRITE_INIT_DELAY:\t%10.2f", avg_write_init_delay);
                $fdisplay(fd0, "MEDIAN_WRITE_INIT_DELAY:\t%10d", median_write_init_delay);

                $fdisplay(fd0, "------------------------------- WRITE_DURATION_STATISTICS -----------------------------");
                $fdisplay(fd0, "MAX_WRITE_DURATION:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                max_write_duration,
                                max_write_duration_time,
                                write_stats_arr[max_write_duration_idx].address,
                                write_stats_arr[max_write_duration_idx].id,
                                max_write_duration_idx);
                $fdisplay(fd0, "MIN_WRITE_DURATION:\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                min_write_duration,
                                min_write_duration_time,
                                write_stats_arr[min_write_duration_idx].address,
                                write_stats_arr[min_write_duration_idx].id,
                                min_write_duration_idx);
                $fdisplay(fd0, "AVG_WRITE_DURATION:\t%10.2f", avg_write_duration);
                $fdisplay(fd0, "MEDIAN_WRITE_DURATION:\t%10d", median_write_duration);

                `ifndef THROUGHPUT_OFF
                    $fdisplay(fd0, "------------------------------- WRITE_THROUGHPUT_STATISTICS ---------------------------");
                    $fdisplay(fd0, "MAX_WRITE_THROUGHPUT:\t\t%5.2f %%, %10d beats / %10d cycles issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                    max_write_throughput*100.0,
                                    write_stats_arr[max_write_throughput_idx].beat_count + write_stats_arr[max_write_throughput_idx].early_beat_count,
                                    write_stats_arr[max_write_throughput_idx].throughput_dnm,
                                    max_write_throughput_time,
                                    write_stats_arr[max_write_throughput_idx].address,
                                    write_stats_arr[max_write_throughput_idx].id,
                                    max_write_throughput_idx);
                    $fdisplay(fd0, "MIN_WRITE_THROUGHPUT:\t\t%5.2f %%, %10d beats / %10d cycles issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                    min_write_throughput*100.0,
                                    write_stats_arr[min_write_throughput_idx].beat_count + write_stats_arr[min_write_throughput_idx].early_beat_count,
                                    write_stats_arr[min_write_throughput_idx].throughput_dnm,
                                    min_write_throughput_time,
                                    write_stats_arr[min_write_latency_idx].address,
                                    write_stats_arr[min_write_latency_idx].id,
                                    min_write_throughput_idx);
                    $fdisplay(fd0, "AVG_WRITE_THROUGHPUT_PER_BURST:\t\t%5.2f %%", avg_write_throughput*100.0);
                    /* $fdisplay(fd0, "AVG_WRITE_THROUGHPUT_OVERALL:\t\t%5.2f %%, %10d beats / %10d cycles", ($itor(snap_wr_total_beat_count)/$itor(snap_wr_total_throughput_dnm_count))*100,
                                                                                                snap_wr_total_beat_count,
                                                                                                snap_wr_total_throughput_dnm_count); */
                    $fdisplay(fd0, "MEDIAN_WRITE_THROUGHPUT:\t%5.2f %%", median_write_throughput*100.0);
                `endif

                $fdisplay(fd0, "------------------------------- WRITE_BACKPRESSURE_STATISTICS -------------------------");
                $fdisplay(fd0, "MAX_WRITE_BACKPRESSURE:\t\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                max_write_backpressure,
                                max_write_backpressure_time,
                                write_stats_arr[max_write_backpressure_idx].address,
                                write_stats_arr[max_write_backpressure_idx].id,
                                max_write_backpressure_idx);
                $fdisplay(fd0, "MIN_WRITE_BACKPRESSURE:\t\t%10d issued at %10d ps, addr: 0x%8h, id: %d, burst no.: %10d",
                                min_write_backpressure,
                                min_write_backpressure_time,
                                write_stats_arr[min_write_backpressure_idx].address,
                                write_stats_arr[min_write_backpressure_idx].id,
                                min_write_backpressure_idx);
                $fdisplay(fd0, "AVG_WRITE_BACKPRESSURE:\t\t%10.2f", avg_write_backpressure);
                $fdisplay(fd0, "MEDIAN_WRITE_BACKPRESSURE:\t%10d", median_write_backpressure);

                $fdisplay(fd0, "------------------------------- TOTAL_WRITE_STATISTICS --------------------------------");
                $fdisplay(fd0, "TOTAL_WRITE_BURSTS:\t\t%10d", snap_wr_total_burst_count);
                $fdisplay(fd0, "TOTAL_WRITE_BEATS:\t\t%10d", snap_wr_total_beat_count);
                $fdisplay(fd0, "TOTAL_WRITE_BUS_ACTIVE_CYCLES:\t%10d", snap_wr_total_bus_active);
                $fdisplay(fd0, "TOTAL_WRITE_BUS_IDLE_CYCLES:\t%10d", snap_wr_total_idle_count);
            end

            if(!((snap_rd_total_burst_count == 0) && (snap_wr_total_burst_count == 0))) begin
                $fdisplay(fd0, "------------------------------- CHANNEL_UTILIZATION -----------------------------------");
                $fdisplay(fd0, "AW_STALLS:\t%10d, AW_VALID:\t%10d", snap_wr_total_addr_stall_count, snap_wr_total_awvalid_count);
                $fdisplay(fd0, "W_STALLS:\t%10d, W_VALID:\t%10d", snap_wr_total_stall_count, snap_wr_total_wvalid_count);
                $fdisplay(fd0, "B_STALLS:\t%10d, B_VALID:\t%10d", snap_wr_total_b_stall_count, snap_wr_total_awvalid_count);
                $fdisplay(fd0, "AR_STALLS:\t%10d, AR_VALID:\t%10d", snap_rd_total_ar_stall_count, snap_rd_total_arvalid_count);
                $fdisplay(fd0, "R_STALLS:\t%10d, R_VALID:\t%10d", snap_rd_total_r_stall_count, snap_rd_total_rvalid_count);
                $fdisplay(fd0, "AW_MAX_ACCEPTANCE_CAPABILITY:\t%10d", wr_aw_max_outstanding);
                $fdisplay(fd0, "W_MAX_ACCEPTANCE_CAPABILITY:\t%10d", wr_w_max_outstanding);
                $fdisplay(fd0, "AR_MAX_ACCEPTANCE_CAPABILITY:\t%10d", rd_max_outstanding);
            end

            $fclose(fd0);
            file_written <= 1'b1;
        end else begin
            $display("Error: Unable to open file %s", $sformatf("%s.txt", OUT_FILE_NAME));
        end

        if (DUMP_RAW_DATA == 1) begin
            if(fd1) begin
                // Write CSV header
                $fwrite(fd1, "address,id,time_issued,time_completed,duration,latency,init_delay,throughput_dnm,burst_length,r_stall_count,slow_link_count,lag_count,ar_stall_count,ar_cycle_count,beat_count\n");

                // Write each entry in read_stats_arr to the CSV file
                for (i = 0; i <= read_stats_last; i++) begin
                    $fwrite(fd1, "%h,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                            read_stats_arr[i].address,
                            read_stats_arr[i].id,
                            read_stats_arr[i].time_issued,
                            read_stats_arr[i].time_completed,
                            read_stats_arr[i].duration,
                            read_stats_arr[i].latency,
                            read_stats_arr[i].init_delay,
                            read_stats_arr[i].throughput_dnm,
                            read_stats_arr[i].burst_length,
                            read_stats_arr[i].r_stall_count,
                            read_stats_arr[i].slow_link_count,
                            read_stats_arr[i].lag_count,
                            read_stats_arr[i].ar_stall_count,
                            read_stats_arr[i].ar_cycle_count,
                            read_stats_arr[i].beat_count);
                end

                $fclose(fd1);
                file_written <= 1'b1;
            end else begin
                $display("Error: Unable to open file %s", $sformatf("%s_read.csv", RAW_OUT_FILE_NAME));
            end

            if (fd2) begin
                // Write CSV header
                $fwrite(fd2, "address,id,time_issued,time_completed,duration,latency,init_delay,throughput_dnm,backpressure,burst_length,stall_count,beat_count,b_lag_count,b_stall_count,b_end_count,early_stall_count,slow_data_count,early_beat_count,addr_lag_count,data_lag_count,awr_early_count,addr_stall_count\n");

                // Write each entry in write_stats_arr to the CSV file
                for (i = 0; i <= write_stats_last; i++) begin
                    $fwrite(fd2, "%h,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                            write_stats_arr[i].address,
                            write_stats_arr[i].id,
                            write_stats_arr[i].time_issued,
                            write_stats_arr[i].time_completed,
                            write_stats_arr[i].duration,
                            write_stats_arr[i].latency,
                            write_stats_arr[i].init_delay,
                            write_stats_arr[i].throughput_dnm,
                            write_stats_arr[i].backpressure,
                            write_stats_arr[i].burst_length,
                            write_stats_arr[i].stall_count,
                            write_stats_arr[i].beat_count,
                            write_stats_arr[i].b_lag_count,
                            write_stats_arr[i].b_stall_count,
                            write_stats_arr[i].b_end_count,
                            write_stats_arr[i].early_stall_count,
                            write_stats_arr[i].slow_data_count,
                            write_stats_arr[i].early_beat_count,
                            write_stats_arr[i].addr_lag_count,
                            write_stats_arr[i].data_lag_count,
                            write_stats_arr[i].awr_early_count,
                            write_stats_arr[i].addr_stall_count);
                end

                $fclose(fd2);
                file_written <= 1'b1;
            end else begin
                $display("Error: Unable to open file %s", $sformatf("%s_write.csv", RAW_OUT_FILE_NAME));
            end

            if (fd3) begin
                // Dump 'total' statistics to CSV file
                // Read statistics
                $fwrite(fd3, "active_cycles,%0d\n", snap_active_count);
                $fwrite(fd3, "read_bursts,%0d\n", snap_rd_total_burst_count);
                $fwrite(fd3, "read_beats,%0d\n", snap_rd_total_beat_count);
                $fwrite(fd3, "read_bus_active_cycles,%0d\n", snap_rd_total_bus_active_count);
                $fwrite(fd3, "read_bus_idle_cycles,%0d\n", snap_rd_total_idle_count);
                $fwrite(fd3, "read_ar_stall_count,%0d\n", snap_rd_total_ar_stall_count);
                $fwrite(fd3, "read_r_stall_count,%0d\n", snap_rd_total_r_stall_count);
                $fwrite(fd3, "read_arvalid_count,%0d\n", snap_rd_total_arvalid_count);
                $fwrite(fd3, "read_rvalid_count,%0d\n", snap_rd_total_rvalid_count);
                $fwrite(fd3, "read_slow_link_count,%0d\n", snap_rd_total_slow_link_count);
                $fwrite(fd3, "read_ar_cycle_count,%0d\n", snap_rd_total_ar_cycle_count);
                $fwrite(fd3, "read_lag_count,%0d\n", snap_rd_total_lag_count);
                $fwrite(fd3, "read_throughput_dnm_count,%0d\n", snap_rd_total_throughput_dnm_count);
                $fwrite(fd3, "read_max_latency,%0d\n", max_read_latency);
                $fwrite(fd3, "read_min_latency,%0d\n", min_read_latency);
                $fwrite(fd3, "read_median_latency,%0d\n", median_read_latency);
                $fwrite(fd3, "read_avg_latency,%0.2f\n", avg_read_latency);
                $fwrite(fd3, "read_max_init_delay,%0d\n", max_read_init_delay);
                $fwrite(fd3, "read_min_init_delay,%0d\n", min_read_init_delay);
                $fwrite(fd3, "read_median_init_delay,%0d\n", median_read_init_delay);
                $fwrite(fd3, "read_avg_init_delay,%0.2f\n", avg_read_init_delay);
                $fwrite(fd3, "read_max_duration,%0d\n", max_read_duration);
                $fwrite(fd3, "read_min_duration,%0d\n", min_read_duration);
                $fwrite(fd3, "read_median_duration,%0d\n", median_read_duration);
                $fwrite(fd3, "read_avg_duration,%0.2f\n", avg_read_duration);
                $fwrite(fd3, "read_max_throughput,%0.4f\n", max_read_throughput);
                $fwrite(fd3, "read_min_throughput,%0.4f\n", min_read_throughput);
                $fwrite(fd3, "read_median_throughput,%0.4f\n", median_read_throughput);
                $fwrite(fd3, "read_avg_throughput,%0.4f\n", avg_read_throughput);
                $fwrite(fd3, "read_max_outstanding,%0d\n", rd_max_outstanding);

                // Write statistics
                $fwrite(fd3, "write_bursts,%0d\n", snap_wr_total_burst_count);
                $fwrite(fd3, "write_beats,%0d\n", snap_wr_total_beat_count);
                $fwrite(fd3, "write_bus_active_cycles,%0d\n", snap_wr_total_bus_active);
                $fwrite(fd3, "write_bus_idle_cycles,%0d\n", snap_wr_total_idle_count);
                $fwrite(fd3, "write_awvalid_count,%0d\n", snap_wr_total_awvalid_count);
                $fwrite(fd3, "write_bvalid_count,%0d\n", snap_wr_total_bvalid_count);
                $fwrite(fd3, "write_wvalid_count,%0d\n", snap_wr_total_wvalid_count);
                $fwrite(fd3, "write_addr_stall_count,%0d\n", snap_wr_total_addr_stall_count);
                $fwrite(fd3, "write_addr_lag_count,%0d\n", snap_wr_total_addr_lag_count);
                $fwrite(fd3, "write_early_stall_count,%0d\n", snap_wr_total_early_stall_count);
                $fwrite(fd3, "write_b_lag_count,%0d\n", snap_wr_total_b_lag_count);
                $fwrite(fd3, "write_b_stall_count,%0d\n", snap_wr_total_b_stall_count);
                $fwrite(fd3, "write_b_end_count,%0d\n", snap_wr_total_b_end_count);
                $fwrite(fd3, "write_data_lag_count,%0d\n", snap_wr_total_data_lag_count);
                $fwrite(fd3, "write_stall_count,%0d\n", snap_wr_total_stall_count);
                $fwrite(fd3, "write_slow_data_count,%0d\n", snap_wr_total_slow_data_count);
                $fwrite(fd3, "write_awr_early_count,%0d\n", snap_wr_total_awr_early_count);
                $fwrite(fd3, "write_early_beat_count,%0d\n", snap_wr_total_early_beat_count);
                $fwrite(fd3, "write_throughput_dnm_count,%0d\n", snap_wr_total_throughput_dnm_count);
                $fwrite(fd3, "write_max_latency,%0d\n", max_write_latency);
                $fwrite(fd3, "write_min_latency,%0d\n", min_write_latency);
                $fwrite(fd3, "write_median_latency,%0d\n", median_write_latency);
                $fwrite(fd3, "write_avg_latency,%0.2f\n", avg_write_latency);
                $fwrite(fd3, "write_max_init_delay,%0d\n", max_write_init_delay);
                $fwrite(fd3, "write_min_init_delay,%0d\n", min_write_init_delay);
                $fwrite(fd3, "write_median_init_delay,%0d\n", median_write_init_delay);
                $fwrite(fd3, "write_avg_init_delay,%0.2f\n", avg_write_init_delay);
                $fwrite(fd3, "write_max_duration,%0d\n", max_write_duration);
                $fwrite(fd3, "write_min_duration,%0d\n", min_write_duration);
                $fwrite(fd3, "write_median_duration,%0d\n", median_write_duration);
                $fwrite(fd3, "write_avg_duration,%0.2f\n", avg_write_duration);
                $fwrite(fd3, "write_max_backpressure,%0d\n", max_write_backpressure);
                $fwrite(fd3, "write_min_backpressure,%0d\n", min_write_backpressure);
                $fwrite(fd3, "write_median_backpressure,%0d\n", median_write_backpressure);
                $fwrite(fd3, "write_avg_backpressure,%0.2f\n", avg_write_backpressure);
                $fwrite(fd3, "write_max_throughput,%0.4f\n", max_write_throughput);
                $fwrite(fd3, "write_min_throughput,%0.4f\n", min_write_throughput);
                $fwrite(fd3, "write_median_throughput,%0.4f\n", median_write_throughput);
                $fwrite(fd3, "write_avg_throughput,%0.4f\n", avg_write_throughput);
                $fwrite(fd3, "write_aw_max_outstanding,%0d\n", wr_aw_max_outstanding);
                $fwrite(fd3, "write_w_max_outstanding,%0d\n", wr_w_max_outstanding);

                $fclose(fd3);
                file_written <= 1'b1;
            end else begin
                $display("Error: Unable to open file %s", $sformatf("%s_total.csv", RAW_OUT_FILE_NAME));
            end
        end
    end
end

`ifndef ASSERTIONS_OFF

    logic [MAXCNT-1:0] wr_burst_saved_counter, wr_burst_bus_counter;
    logic [MAXCNT-1:0] rd_burst_saved_counter, rd_burst_bus_counter;

    always_ff @(posedge clk or negedge nres) begin : aux_burst_counter
        if (!nres) begin
            wr_burst_saved_counter <= '0;
            wr_burst_bus_counter   <= '0;
            rd_burst_saved_counter <= '0;
            rd_burst_bus_counter   <= '0;
        end else begin
            if (wr_recording_saved) begin
                wr_burst_saved_counter <= wr_burst_saved_counter + 1;
            end
            if (axi_bus.aw_valid && axi_bus.aw_ready) begin
                wr_burst_bus_counter <= wr_burst_bus_counter + 1;
            end
            if (rd_recording_saved) begin
                rd_burst_saved_counter <= rd_burst_saved_counter + 1;
            end
            if (axi_bus.ar_valid && axi_bus.ar_ready) begin
                rd_burst_bus_counter <= rd_burst_bus_counter + 1;
            end
        end
    end

    // Assertions to check if the burst counters match
    wr_number_saved_bursts_matches_bursts_on_bus:   assert property ( stop_measurement |-> wr_burst_saved_counter == wr_burst_bus_counter );
    rd_number_saved_bursts_matches_bursts_on_bus:   assert property ( stop_measurement |-> rd_burst_saved_counter == rd_burst_bus_counter );


    // Assertions to check that only exactly one counter of each active recording in the write_stats_matrix is incremented each cycle -> each bus cycle needs to be binned
    function automatic logic wr_no_increment_any_set();
        wr_no_increment_any_set = 1'b0;
        foreach (wr_no_increment[i, j]) begin
            wr_no_increment_any_set |= wr_no_increment[i][j];
        end
    endfunction

    property WR_MATRIX_ALWAYS_SINGLE_COUNTER_INCREMENT_EACH_ACTIVE_CYCLE;
        !wr_no_increment_any_set(); // if any bit in the 2D array is set, the property fails
    endproperty

    assert property ( WR_MATRIX_ALWAYS_SINGLE_COUNTER_INCREMENT_EACH_ACTIVE_CYCLE );

    // Assertions to check that only exactly one counter of each active recording in the temp_stats_arr is incremented each cycle -> each bus cycle needs to be binned
    function automatic logic temp_wr_no_increment_any_set();
        temp_wr_no_increment_any_set = 1'b0;
        foreach (temp_wr_no_increment[i]) begin
            temp_wr_no_increment_any_set |= temp_wr_no_increment[i];
        end
    endfunction

    property TEMP_WR_MATRIX_ALWAYS_SINGLE_COUNTER_INCREMENT_EACH_ACTIVE_CYCLE;
        !temp_wr_no_increment_any_set(); // if any bit in the 2D array is set, the property fails
    endproperty

    assert property ( TEMP_WR_MATRIX_ALWAYS_SINGLE_COUNTER_INCREMENT_EACH_ACTIVE_CYCLE );

    property MERGE_WR_MATRIX_ALWAYS_SINGLE_COUNTER_INCREMENT;
        !merge_wr_no_increment;
    endproperty

    assert property ( MERGE_WR_MATRIX_ALWAYS_SINGLE_COUNTER_INCREMENT );

    // Assertions to check that only exactly one counter of each active recording in the read_stats_matrix is incremented each cycle -> each bus cycle needs to be binned
    function automatic logic rd_no_increment_any_set();
        rd_no_increment_any_set = 1'b0;
        foreach (rd_no_increment[i, j]) begin
            rd_no_increment_any_set |= rd_no_increment[i][j];
        end
    endfunction

    property RD_MATRIX_ALWAYS_SINGLE_COUNTER_INCREMENT_EACH_ACTIVE_CYCLE;
       !rd_no_increment_any_set();   // if any bit in the 2D array is set, the property fails
    endproperty

    assert property ( RD_MATRIX_ALWAYS_SINGLE_COUNTER_INCREMENT_EACH_ACTIVE_CYCLE );
`endif

`ifdef ASSERTIONS_OFF
`undef ASSERTIONS_OFF
`endif

`ifdef THROUGHPUT_OFF
`undef THROUGHPUT_OFF
`endif
endmodule
