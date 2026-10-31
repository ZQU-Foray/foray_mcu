#ifndef ST_STM32H7_BOARD_DM_PORT_INIT_H
#define ST_STM32H7_BOARD_DM_PORT_INIT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
 * @brief 初始化当前目标的全部 Port
 * @return 全部 Port 初始化成功时返回 true，否则返回 false
 */
    bool Port_Init(void);

#ifdef __cplusplus
}
#endif

#endif
