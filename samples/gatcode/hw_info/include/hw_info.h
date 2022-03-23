#ifndef HW_INFO
#define HW_INFO

/**
 * @brief Get the device specific ID
 * 
 * This function retrieves the device specific ID and stores
 * the result in the handed over uint64_t variable.
 *
 * @param id Pointer to a uint64_t variable to store the retrieved id.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int get_device_id(uint64_t *id);

#endif