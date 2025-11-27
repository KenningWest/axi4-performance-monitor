# axi4-performance-monitor
This AXI4 Performance Monitor observes AXI transactions on its AXI4 port, records detailed statistical data about the bursts in flight and writes them to a file. Extended and modified version of https://github.com/ZipCPU/wb2axip/blob/master/rtl/axiperf.v

![axi_perfmonitor_block_diagram](https://github.com/user-attachments/assets/828f0e39-cafb-4119-90c5-8e25c44fcc22)

For use in testbench environment only!

Important notice: Behaviour of this monitor for the case that first WVALID precedes the first AWVALID cycle of a burst is implemented but not thorougly tested! If it occurs a message will be included in the output summary file and an info message is printed to the console.

## Requirements
This design uses the AXI_BUS interface from the PULP Platform. Please refer to https://github.com/pulp-platform/axi/blob/master/src/axi_intf.sv

## Parameters

| Parameter | Description |
|-----------|-------------|
| `MAX_NUM_REC_BURSTS` | Size of buffer holding recorded bursts (default: 10000). |
| `OUT_FILE_NAME` | Output file name for summary text file. |
| `RAW_OUT_FILE_NAME` | Output file name for raw data files. |
| `DUMP_RAW_DATA` | Set to 1 to dump raw data to a CSV file, 0 to disable. |
| `MANUAL_CONTROL` | Determines whether measurement shall be started with first transfer on bus and stopped with stopclock signal (0) or by external start/stop signal (1). |
| `FILTER_ADDR` | Address for which recorded statistics of transfers are omitted (start address only). |
| `FILTER_MASK` | Bitmask determining which address bits are considered in the filter. |
| `FILTER_EN` | Address filter enable bit. |

Additional parameters for configuring the monitor's AXI interface

## Recording Modes
The monitor supports two recording modes:
- Automatic: Recording starts with the first burst after reset and ends with the stopclock signal. The output file is written automatically at the end of recording.
- Manual: Recording starts with the `manual_start` signal, stops with `manual_stop`, clears internal buffers with `manual_clear`, and writes the output with `manual_print`.

The mode is selected via the MANUAL_CONTROL parameter (0 = automatic, 1 = manual).

IMPORTANT: in the current state, if measurement is stopped too soon after the last burst, it may happen that the last burst is not recorded! Also if stopped in the middle of a burst, this burst is not recorded! This is because the monitor only outputs statistics for bursts that are completely finished, i.e. WLAST or RLAST has occured and the recording had time to be saved (normally 2 cycles afterwards)
-> as a safety margin 5 clock cycles between the last relevant bursts finishing (i.e. WLAST or RLAST) and measurement stop is recommended. For auto mode, this means delaying the stopclock signal by at least 5 clock cycles

## Assertions
This design includes several assertions ensuring that for all bursts currently in flight only one counter is increased each cycle (only one bin is assigned per burst per cycle). They can be turned off by defining `ASSERTIONS_OFF`.

## Clock Cycle Binning
The cycle binning logic is largely derived from the original design, but adapted to allow for individual tracking of multiple bursts simultaneously instead of assigning a single bin for the entire bus. For more details refer to https://zipcpu.com/blog/2021/08/14/axiperf.html 

### Read Cycle Binning Logic
<img width="1887" height="463" alt="AXI_perfmonitor_read_cycle_binning" src="https://github.com/user-attachments/assets/1f6570bc-b87f-48fa-8274-c5266f84b88f" />

### Write Cycle Binning
<img width="2373" height="825" alt="AXI_perfmonitor_write_cycle_binning" src="https://github.com/user-attachments/assets/41aaee42-dfa1-487b-86e1-c22c93ae2335" />

### Temp Write Cycle Binning
In AXI4, because of the decoupled nature of the write data and address channel, write data can precede its associated address. Due to the structure of the write burst recording matrix of this monitor being organized with respect to transaction IDs, which are only available once the start address is sent, these write bursts are buffered temporarily. Cycle binning differs from regular write cycles, as detailed in the following table:
<img width="1673" height="430" alt="AXI_perfmonitor_temp_write_cycle_binning" src="https://github.com/user-attachments/assets/6f9ab8a7-fde3-4ac0-bcff-9d819701ee20" />

## Recorded Metrics
### Read Statistics
- Read Latency
- Read Initiation Delay
- Read Duration
- Read Throughput
- Total Read Statistics:
    * Total \# Read Bursts
    * Total \# Read Beats
    * Total \# Read Channel Active Cycles
    * Total \# Read Channel Idle Cycles

### Write Statistics
- Write Latency
- Write Initiation Delay
- Write Duration
- Write Throughput
- Write Backpressure
- Total Write Statistics:
    * Total \# Write Bursts
    * Total \# Write Beats
    * Total \# Write Channel Active Cycles
    * Total \# Write Channel Idle Cycles

### Channel utilization
- AW Stalls & AWVALID Cycles
- W Stalls & WVALID Cycles
- B Stalls & BVALID Cycles
- AR Stalls & ARVALID Cycles
- R Stalls & RVALID Cycles

For all statistics (except total statistics and channel utilization), max, min, avg, and median values are calculated and written to the summary file.

## Known Issues
- The snapshot total statistics include the cycle after a burst has finished. This is caused by triggering on wr/rd_recording_saved signals, which lag one cycle behind the actual burst completion.
- rd_total_r_stall_count, rd_total_slow_link_count, rd_total_lag_count, and rd_total_ar_stall_count are not updated correctly, because e.g. r_rd_total_r_stall_count does not get transferred the value from read_stats_matrix[id][0].r_stall_count for some reason
- The monitor is based around a static 2D array of structs (read_stats_matrix/write_stats_matrix) that stores the information about outstanding read and write bursts. Whenever a burst is completed, the first element of the matrix is popped and all subsequent elements are shifted down by one -> function read/write_matrix_pop. This leads to a problem, when in the same cycle either of the functions read/write_burst_record_completion_time are called, since they write to the element at index 0 as well as read/write_matrix_pop. This is currently circumvented by not copying the time_completed value in the read/write_matrix_pop function. A better solution would be to use a queue instead of a 2D array or to keep the matrix static and use pointers to indicate the order of transactions inside of the matrix. This would also safeguard against any interference between the counter update logic and the pop logic.
