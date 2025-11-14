#ifndef __TUSBH_XBOX_H__
#define __TUSBH_XBOX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tusbh.h"

typedef struct _tusbh_xbox_class
{
    const tusbh_interface_backend_t* backend;    
    int(*on_data)(tusbh_ep_info_t* ep, const uint8_t* data);
}tusbh_xbox_class_t;

extern const uint16_t xbox_id_table;

int tusbh_vendor_xfer_data(tusbh_ep_info_t* ep, void* data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif