/**
 * Manages the data stream from the microphone on the Magnet Module. The
 * data is received over the I2S bus.
 *
 * In order for reads to be successful, the initialize and enable methods must
 * be called first.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

/**
 * Enable/Disable reading on the I2S bus.
 *
 * @param device - Microphone device; Can not be NULL
 * @param enable
 */
[[nodiscard]] bool ll_microphone_enable_reads(const struct device* device, bool enable);

/**
 * Read (blocking) a data set from the microphone. On success, mem_block and block_size
 * are updated with linkage to the read data.
 *
 * After any processing is completed, app MUST call mic_release_buffer on the
 * return memory block.
 *
 * @param device - Microphone device; Can not be NULL
 * @param[out] mem_block - Non-NULL pointer, will contain pointer to read data
 * @param[out] block_size - Non-NULL pointer, will contain the number of samples read across all channels
 *
 * @retval 0 - OK
 * @retval -EAGAIN - No data available
 * @retval <0 - Error code
 */
[[nodiscard]] int ll_microphone_read(const struct device* device, void** mem_block, uint32_t* block_size);

/**
 * Releases the buffer to allow the microphone system to re-use the block.
 *
 * WARNING: if buffers are not released, no more data will become available.
 *
 * @param device - Microphone device; Can not be NULL
 * @param mem_block - Memory block to release back to the kernel for re-use
 */
void ll_microphone_release_buffer(const struct device* device, void* mem_block);

/**
 *
 * @param device - Microphone device; Can not be NULL
 *
 * @return Number of input channels
 */
int ll_microphone_channel_count(const struct device* device);


/**
 * Perform a flush and restart of the device reading. Returns 0 on success, else error code (-errno)
 *
 * @param device - Microphone device; Can not be NULL
 *
 * @return Number of input channels
 */
int ll_microphone_reset_read(const struct device* device);
