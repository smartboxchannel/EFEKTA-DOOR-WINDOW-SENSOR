#ifndef APP_GPIOTE_H__
#define APP_GPIOTE_H__

#include <stdbool.h>
#include "nrf.h"
#include "app_util.h"

#define GPIOTE_USER_NODE_SIZE   20          /**< Size of app_gpiote.gpiote_user_t (only for use inside APP_GPIOTE_BUF_SIZE()). */
#define NO_OF_PINS              32          /**< Number of GPIO pins on the nRF51 chip. */

/**@brief Compute number of bytes required to hold the GPIOTE data structures.
 *
 * @param[in]   MAX_USERS   Maximum number of GPIOTE users.
 *
 * @return      Required buffer size (in bytes).
 */
#define APP_GPIOTE_BUF_SIZE(MAX_USERS)  ((MAX_USERS) * GPIOTE_USER_NODE_SIZE)

typedef uint8_t app_gpiote_user_id_t;

/**@brief GPIOTE event handler type. */
typedef void (*app_gpiote_event_handler_t)(uint32_t event_pins_low_to_high,
                                           uint32_t event_pins_high_to_low);

/**@brief GPIOTE input event handler type. */
typedef void (*app_gpiote_input_event_handler_t)(void);

/**@brief Macro for initializing the GPIOTE module.
 *
 * @details It will handle dimensioning and allocation of the memory buffer required by the module,
 *          making sure that the buffer is correctly aligned.
 *
 * @param[in]   MAX_USERS   Maximum number of GPIOTE users.
 *
 * @note Since this macro allocates a buffer, it must only be called once (it is OK to call it
 *       several times as long as it is from the same location, e.g. to do a reinitialization).
 */
/*lint -emacro(506, APP_GPIOTE_INIT) */ /* Suppress "Constant value Boolean */
#define APP_GPIOTE_INIT(MAX_USERS)                                                                 \
    do                                                                                             \
    {                                                                                              \
        static uint32_t app_gpiote_buf[CEIL_DIV(APP_GPIOTE_BUF_SIZE(MAX_USERS), sizeof(uint32_t))];\
        app_gpiote_init((MAX_USERS), app_gpiote_buf);                          \
    } while (0)

/**@brief Function for initializing the GPIOTE module.
 *
 * @note Normally initialization should be done using the APP_GPIOTE_INIT() macro, as that will
 *       allocate the buffer needed by the GPIOTE module (including aligning the buffer correctly).
 *
 * @param[in]   max_users               Maximum number of GPIOTE users.
 * @param[in]   p_buffer                Pointer to memory buffer for internal use in the app_gpiote
 *                                      module. The size of the buffer can be computed using the
 *                                      APP_GPIOTE_BUF_SIZE() macro. The buffer must be aligned to 
 *                                      a 4 byte boundary.
 *
 * @retval      NRF_SUCCESS             Successful initialization.
 * @retval      NRF_ERROR_INVALID_PARAM Invalid parameter (buffer not aligned to a 4 byte
 *                                      boundary).
 */
void app_gpiote_init(uint8_t max_users, void * p_buffer);

/**@brief Function for registering a GPIOTE user.
 *
 * @param[out]  p_user_id               Id for the new GPIOTE user.
 * @param[in]   pins_low_to_high_mask   Mask defining which pins will generate events to this user 
 *                                      when state is changed from low->high.
 * @param[in]   pins_high_to_low_mask   Mask defining which pins will generate events to this user
 *                                      when state is changed from high->low.
 * @param[in]   event_handler           Pointer to function to be executed when an event occurs.
 *
 * @retval      NRF_SUCCESS             Successful initialization.
 * @retval      NRF_ERROR_INVALID_PARAM Invalid parameter (buffer not aligned to a 4 byte boundary).
 * @retval      NRF_ERROR_INALID_STATE  If @ref app_gpiote_init has not been called on the GPIOTE
 *                                      module.
 * @retval      NRF_ERROR_NO_MEM        Returned if the application tries to register more users
 *                                      than defined when the GPIOTE module was initialized in
 *                                      @ref app_gpiote_init.
 */
void app_gpiote_user_register(app_gpiote_user_id_t *     p_user_id,
                                  uint32_t                   pins_low_to_high_mask,
                                  uint32_t                   pins_high_to_low_mask,
                                  app_gpiote_event_handler_t event_handler);

/**@brief Function for informing the GPIOTE module that the specified user wants to use the GPIOTE module.
 *
 * @param[in]   user_id                 Id of user to enable.
 *
 * @retval      NRF_SUCCESS             On success.
 * @retval      NRF_ERROR_INVALID_PARAM Invalid user_id provided, No a valid user.
 * @retval      NRF_ERROR_INALID_STATE  If @ref app_gpiote_init has not been called on the GPIOTE
 *                                      module.
 */
void app_gpiote_user_enable(app_gpiote_user_id_t user_id);

/**@brief Function for informing the GPIOTE module that the specified user is done using the GPIOTE module.
 *
 * @param[in]   user_id                   Id of user to enable.
 *
 * @return      NRF_SUCCESS               On success.
 * @retval      NRF_ERROR_INVALID_PARAM   Invalid user_id provided, No a valid user.
 * @retval      NRF_ERROR_INALID_STATE    If @ref app_gpiote_init has not been called on the GPIOTE
 *                                        module.
 */
void app_gpiote_user_disable(app_gpiote_user_id_t user_id);


#endif // APP_GPIOTE_H__

/** @} */
