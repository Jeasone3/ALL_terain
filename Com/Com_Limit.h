#ifndef __COM_LIMIT_H__
#define __COM_LIMIT_H__

#include "main.h"

/**
 * @brief 限制值在最小值和最大值之间
 * 
 * @note 可以传任意类型的值，但是必须保证类型一致，否则会报错
 *        不要传带副作用的例如 x++
 * @param val 要限制的值 
 *          
 * 
 */
#define Com_Limit(val, min, max) \
    ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))




#endif /* __COM_LIMIT_H__ */
