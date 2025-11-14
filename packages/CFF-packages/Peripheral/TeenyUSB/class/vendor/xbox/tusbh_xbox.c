#include "tusbh_xbox.h"
#include "tusbh_hid.h"
#include "string.h"

#define TUSB_LOGD(fmt, ...)  TUSB_LOGD("[XBOX ] " fmt, ##__VA_ARGS__)

const uint16_t xbox_id_table[][2] = {
    { 0x0079, 0x18d4 }, // GPD Win 2 X-Box Controller
    { 0x03eb, 0xff01 }, // Wooting One (Legacy)
    { 0x03eb, 0xff02 }, // Wooting Two (Legacy)
    { 0x044f, 0xb326 }, // Thrustmaster Gamepad GP XID
    { 0x045e, 0x028e }, // Microsoft X-Box 360 pad
    { 0x045e, 0x028f }, // Microsoft X-Box 360 pad v2
    { 0x046d, 0xc21d }, // Logitech Gamepad F310
    { 0x046d, 0xc21e }, // Logitech Gamepad F510
    { 0x046d, 0xc21f }, // Logitech Gamepad F710
    { 0x046d, 0xc242 }, // Logitech Chillstream Controller
    { 0x046d, 0xcaa3 }, // Logitech DriveFx Racing Wheel
    { 0x056e, 0x2004 }, // Elecom JC-U3613M
    { 0x06a3, 0xf51a }, // Saitek P3600
    { 0x0738, 0x4716 }, // Mad Catz Wired Xbox 360 Controller
    { 0x0738, 0x4718 }, // Mad Catz Street Fighter IV FightStick SE
    { 0x0738, 0x4726 }, // Mad Catz Xbox 360 Controller
    { 0x0738, 0x4736 }, // Mad Catz MicroCon Gamepad
    { 0x0738, 0x4740 }, // Mad Catz Beat Pad
    { 0x0738, 0x9871 }, // Mad Catz Portable Drum
    { 0x0738, 0xb726 }, // Mad Catz Xbox controller - MW2
    { 0x0738, 0xbeef }, // Mad Catz JOYTECH NEO SE Advanced GamePad
    { 0x0738, 0xcb02 }, // Saitek Cyborg Rumble Pad - PC/Xbox 360
    { 0x0738, 0xcb03 }, // Saitek P3200 Rumble Pad - PC/Xbox 360
    { 0x0738, 0xcb29 }, // Saitek Aviator Stick AV8R02
    { 0x0738, 0xf738 }, // Super SFIV FightStick TE S
    { 0x07ff, 0xffff }, // Mad Catz GamePad
    { 0x0e6f, 0x0113 }, // Afterglow AX.1 Gamepad for Xbox 360
    { 0x0e6f, 0x011f }, // Rock Candy Gamepad Wired Controller
    { 0x0e6f, 0x0131 }, // PDP EA Sports Controller
    { 0x0e6f, 0x0133 }, // Xbox 360 Wired Controller
    { 0x0e6f, 0x0201 }, // Pelican PL-3601 'TSZ' Wired Xbox 360 Controller
    { 0x0e6f, 0x0213 }, // Afterglow Gamepad for Xbox 360
    { 0x0e6f, 0x021f }, // Rock Candy Gamepad for Xbox 360
    { 0x0e6f, 0x0301 }, // Logic3 Controller
    { 0x0e6f, 0x0401 }, // Logic3 Controller
    { 0x0e6f, 0x0413 }, // Afterglow AX.1 Gamepad for Xbox 360
    { 0x0e6f, 0x0501 }, // PDP Xbox 360 Controller
    { 0x0e6f, 0xf900 }, // PDP Afterglow AX.1
    { 0x0f0d, 0x000a }, // Hori Co. DOA4 FightStick
    { 0x0f0d, 0x000c }, // Hori PadEX Turbo
    { 0x1038, 0x1430 }, // SteelSeries Stratus Duo
    { 0x1038, 0x1431 }, // SteelSeries Stratus Duo
    { 0x11c9, 0x55f0 }, // Nacon GC-100XF
    { 0x1209, 0x2882 }, // Ardwiino Controller
    { 0x12ab, 0x0301 }, // PDP AFTERGLOW AX.1
    { 0x1430, 0x4748 }, // RedOctane Guitar Hero X-plorer
    { 0x1430, 0xf801 }, // RedOctane Controller
    { 0x146b, 0x0601 }, // BigBen Interactive XBOX 360 Controller
    { 0x1532, 0x0037 }, // Razer Sabertooth
    { 0x15e4, 0x3f00 }, // Power A Mini Pro Elite
    { 0x15e4, 0x3f0a }, // Xbox Airflo wired controller
    { 0x15e4, 0x3f10 }, // Batarang Xbox 360 controller
    { 0x162e, 0xbeef }, // Joytech Neo-Se Take2
    { 0x1689, 0xfd00 }, // Razer Onza Tournament Edition
    { 0x1689, 0xfd01 }, // Razer Onza Classic Edition
    { 0x1689, 0xfe00 }, // Razer Sabertooth
    { 0x1949, 0x041a }, // Amazon Game Controller
    { 0x1bad, 0x0002 }, // Harmonix Rock Band Guitar
    { 0x1bad, 0xf016 }, // Mad Catz Xbox 360 Controller
    { 0x1bad, 0xf021 }, // Mad Cats Ghost Recon FS GamePad
    { 0x1bad, 0xf023 }, // MLG Pro Circuit Controller (Xbox)
    { 0x1bad, 0xf025 }, // Mad Catz Call Of Duty
    { 0x1bad, 0xf027 }, // Mad Catz FPS Pro
    { 0x1bad, 0xf028 }, // Street Fighter IV FightPad
    { 0x1bad, 0xf030 }, // Mad Catz Xbox 360 MC2 MicroCon Racing Wheel
    { 0x1bad, 0xf036 }, // Mad Catz MicroCon GamePad Pro
    { 0x1bad, 0xf038 }, // Street Fighter IV FightStick TE
    { 0x1bad, 0xf501 }, // HoriPad EX2 Turbo
    { 0x1bad, 0xf506 }, // Hori Real Arcade Pro.EX Premium VLX
    { 0x1bad, 0xf900 }, // Harmonix Xbox 360 Controller
    { 0x1bad, 0xf901 }, // Gamestop Xbox 360 Controller
    { 0x1bad, 0xf903 }, // Tron Xbox 360 controller
    { 0x1bad, 0xf904 }, // PDP Versus Fighting Pad
    { 0x1bad, 0xfa01 }, // MadCatz GamePad
    { 0x1bad, 0xfd00 }, // Razer Onza TE
    { 0x1bad, 0xfd01 }, // Razer Onza
    { 0x20d6, 0x2001 }, // BDA Xbox Series X Wired Controller
    { 0x20d6, 0x281f }, // PowerA Wired Controller For Xbox 360
    { 0x24c6, 0x5300 }, // PowerA MINI PROEX Controller
    { 0x24c6, 0x5303 }, // Xbox Airflo wired controller
    { 0x24c6, 0x530a }, // Xbox 360 Pro EX Controller
    { 0x24c6, 0x531a }, // PowerA Pro Ex
    { 0x24c6, 0x5397 }, // FUS1ON Tournament Controller
    { 0x24c6, 0x5500 }, // Hori XBOX 360 EX 2 with Turbo
    { 0x24c6, 0x5501 }, // Hori Real Arcade Pro VX-SA
    { 0x24c6, 0x5506 }, // Hori SOULCALIBUR V Stick
    { 0x24c6, 0x550d }, // Hori GEM Xbox controller
    { 0x24c6, 0x5b00 }, // ThrustMaster Ferrari 458 Racing Wheel
    { 0x24c6, 0x5b02 }, // Thrustmaster, Inc. GPX Controller
    { 0x24c6, 0x5b03 }, // Thrustmaster Ferrari 458 Racing Wheel
    { 0x24c6, 0x5d04 }, // Razer Sabertooth
    { 0x24c6, 0xfafe }, // Rock Candy Gamepad for Xbox 360
    { 0x2563, 0x058d }, // OneXPlayer Gamepad
    { 0x2dc8, 0x3106 }, // 8BitDo Ultimate Wireless / Pro 2 Wired Controller
    { 0x2dc8, 0x3109 }, // 8BitDo Ultimate Wireless Bluetooth
    { 0x31e3, 0x1100 }, // Wooting One
    { 0x31e3, 0x1200 }, // Wooting Two
    { 0x31e3, 0x1210 }, // Wooting Lekker
    { 0x31e3, 0x1220 }, // Wooting Two HE
    { 0x31e3, 0x1230 }, // Wooting Two HE (ARM)
    { 0x31e3, 0x1300 }, // Wooting 60HE (AVR)
    { 0x31e3, 0x1310 }, // Wooting 60HE (ARM)
    { 0x3285, 0x0607 }, // Nacon GC-100
    { 0x413d, 0x2104 }, // Black Shark Green Ghost Gamepad
    { 0x0000, 0x0000 }  // end of list
};

static int tusbh_on_boot_xbox(tusbh_ep_info_t* ep, const uint8_t* data)
{
    TUSB_LOGD("data len = %d;",ep->data_len);
    for (uint8_t i=0; i<ep->data_len; i++)
    {
        TUSB_LOGD(" %2x", ((char*)(ep->data))[i]);
    }
    TUSB_LOGD("\n");
    return 0;
}

static int xbox_xfered(tusbh_ep_info_t* ep)
{
    tusbh_hid_info_t* info = tusbh_get_info(ep->interface, tusbh_hid_info_t);
    tusb_hc_data_t* hc = &ep_host(ep)->hc[ep->pipe_num];
    tusbh_device_t* dev = ep_device(ep);
    if(hc->state != TUSB_CS_TRANSFER_COMPLETE){
        return -1;
    }
    
    if(ep->desc->bEndpointAddress != info->ep_in->desc->bEndpointAddress){
        TUSB_HID_LOGW("HID KBD Wrong ep xfered handler, espect %02x, got %02x\n", info->ep_in->desc->bEndpointAddress, ep->desc->bEndpointAddress);
        return -1;
    }
    
    if(ep_class(ep,tusbh_xbox_class_t)->on_data){
        ep_class(ep,tusbh_xbox_class_t)->on_data(ep, (uint8_t*)ep->data);
    }else{
        tusbh_on_boot_xbox(ep, (uint8_t*)ep->data);
    }
    
    return 0;
}

const tusbh_interface_backend_t  tusbh_xbox_backend = {
    .vid = 0,
    .pid = 0,
    .bInterfaceClass = USB_CLASS_VEND_SPECIFIC,
    .bInterfaceSubClass = 0x5d,
    .bInterfaceProtocol = 0x01,
    .init = tusbh_hid_init,
    .deinit = tusbh_hid_deinit,
    .data_xfered = xbox_xfered,
    .desc = "Xbox controller class",
};
