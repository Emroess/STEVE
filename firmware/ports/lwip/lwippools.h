/**
 * @file lwippools.h
 * Custom memory pools for lwIP when MEM_USE_POOLS=1 and MEMP_USE_CUSTOM_POOLS=1
 */

#ifndef __LWIPPOOLS_H__
#define __LWIPPOOLS_H__

/* Define custom memory pools for static allocation */
LWIP_MALLOC_MEMPOOL_START
LWIP_MALLOC_MEMPOOL(20, 256)
LWIP_MALLOC_MEMPOOL(10, 512)
LWIP_MALLOC_MEMPOOL(5, 1024)
LWIP_MALLOC_MEMPOOL(3, 1536)
LWIP_MALLOC_MEMPOOL_END

#endif /* __LWIPPOOLS_H__ */