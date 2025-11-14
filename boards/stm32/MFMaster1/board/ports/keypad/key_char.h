#ifndef _KEY_CHAR_H_
#define _KEY_CHAR_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define VK_BASE     (0x1000)

/* Virtual Keys, Standard Set */
#define VK_LBUTTON  (VK_BASE + 1)   //鼠标的左键
#define VK_RBUTTON  (VK_BASE + 2)   //鼠标的右键
#define VK_CANCEL   (VK_BASE + 3)   //Ctrl+Break(通常不需要处理)
#define VK_MBUTTON  (VK_BASE + 4)   //鼠标的中键(三按键鼠标)
#define VK_BACK     (VK_BASE + 8)   //Backspace键
#define VK_TAB      (VK_BASE + 9)   //Tab键
#define VK_CLEAR    (VK_BASE + 12)  //Clear键(Num Lock关闭时的数字键盘5)
#define VK_RETURN	(VK_BASE + 13)  //Enter键
#define VK_SHIFT    (VK_BASE + 16)  //Shift键
#define VK_CONTROL  (VK_BASE + 17)  //Ctrl键
#define VK_MENU     (VK_BASE + 18)  //Alt键
#define VK_PAUSE    (VK_BASE + 19)  //Pause键
#define VK_CAPITAL  (VK_BASE + 20)  //Caps Lock键
#define VK_ESCAPE   (VK_BASE + 27)  //Ese键
#define VK_SPACE    (VK_BASE + 32)  //Spacebar键
#define VK_PRIOR    (VK_BASE + 33)  //Page Up键
#define VK_NEXT     (VK_BASE + 34)  //Page Domw键
#define VK_END      (VK_BASE + 35)  //End键
#define VK_HOME     (VK_BASE + 36)  //Home键
#define VK_LEFT     (VK_BASE + 37)  //LEFT ARROW 键(←)
#define VK_UP       (VK_BASE + 38)  //UP ARROW键(↑)
#define VK_RIGHT    (VK_BASE + 39)  //RIGHT ARROW键(→)
#define VK_DOWN     (VK_BASE + 40)  //DOWN ARROW键(↓)
#define VK_SELECT   (VK_BASE + 41)  //Select键
#define VK_PRINT    (VK_BASE + 42)
#define VK_EXECUTE  (VK_BASE + 43)  //EXECUTE键
#define VK_SNAPSHOT (VK_BASE + 44)  //Print Screen键（抓屏）
#define VK_INSERT   (VK_BASE + 45)  //Ins键(Num Lock关闭时的数字键盘0)
#define VK_DELETE   (VK_BASE + 46)  //Del键(Num Lock关闭时的数字键盘.)
#define VK_HELP     (VK_BASE + 47)  //Help键
#define VK_0        (VK_BASE + 48)  //0键
#define VK_1        (VK_BASE + 49)  //1键
#define VK_2        (VK_BASE + 50)  //2键
#define VK_3        (VK_BASE + 51)  //3键
#define VK_4        (VK_BASE + 52)  //4键
#define VK_5        (VK_BASE + 53)  //5键
#define VK_6        (VK_BASE + 54)  //6键
#define VK_7        (VK_BASE + 55)  //7键
#define VK_8        (VK_BASE + 56)  //8键
#define VK_9        (VK_BASE + 67)  //9键
#define VK_A        (VK_BASE + 65)  //A键
#define VK_B        (VK_BASE + 66)  //B键
#define VK_C        (VK_BASE + 67)  //C键
#define VK_D        (VK_BASE + 68)  //D键
#define VK_E        (VK_BASE + 69)  //E键
#define VK_F        (VK_BASE + 70)  //F键
#define VK_G        (VK_BASE + 71)  //G键
#define VK_H        (VK_BASE + 72)  //H键
#define VK_I        (VK_BASE + 73)  //I键
#define VK_J        (VK_BASE + 74)  //J键
#define VK_K        (VK_BASE + 75)  //K键
#define VK_L        (VK_BASE + 76)  //L键
#define VK_M        (VK_BASE + 77)  //M键
#define VK_N        (VK_BASE + 78)  //N键
#define VK_O        (VK_BASE + 79)  //O键
#define VK_P        (VK_BASE + 80)  //P键
#define VK_Q        (VK_BASE + 81)  //Q键
#define VK_R        (VK_BASE + 82)  //R键
#define VK_S        (VK_BASE + 83)  //S键
#define VK_T        (VK_BASE + 84)  //T键
#define VK_U        (VK_BASE + 85)  //U键
#define VK_V        (VK_BASE + 86)  //V键
#define VK_W        (VK_BASE + 87)  //W键
#define VK_X        (VK_BASE + 88)  //X键
#define VK_Y        (VK_BASE + 89)  //Y键
#define VK_Z        (VK_BASE + 90)  //Z键
#define VK_NUMPAD0  (VK_BASE + 96)  //数字键0键
#define VK_NUMPAD1  (VK_BASE + 97)  //数字键1键
#define VK_NUMPAD2  (VK_BASE + 98)  //数字键2键
#define VK_NUMPAD3  (VK_BASE + 99)  //数字键3键
#define VK_NUMPAD4  (VK_BASE + 100) //数字键4键
#define VK_NUMPAD5  (VK_BASE + 101) //数字键5键
#define VK_NUMPAD6  (VK_BASE + 102) //数字键6键
#define VK_NUMPAD7  (VK_BASE + 103) //数字键7键
#define VK_NUMPAD8  (VK_BASE + 104) //数字键8键
#define VK_NUMPAD9  (VK_BASE + 105) //数字键9键
#define VK_MULTIPLY (VK_BASE + 106) //数字键盘上的*键
#define VK_ADD      (VK_BASE + 107) //数字键盘上的+键
#define VK_SEPARATOR (VK_BASE + 108) //Separator键
#define VK_SUBTRACT (VK_BASE + 109) //数字键盘上的-键
#define VK_DECIMAL  (VK_BASE + 110) //数字键盘上的.键
#define VK_DIVIDE   (VK_BASE + 111) //数字键盘上的/键
#define VK_F1       (VK_BASE + 112) //F1键
#define VK_F2       (VK_BASE + 113) //F2键
#define VK_F3       (VK_BASE + 114) //F3键
#define VK_F4       (VK_BASE + 115) //F4键
#define VK_F5       (VK_BASE + 116) //F5键
#define VK_F6       (VK_BASE + 117) //F6键
#define VK_F7       (VK_BASE + 118) //F7键
#define VK_F8       (VK_BASE + 119) //F8键
#define VK_F9       (VK_BASE + 120) //F9键
#define VK_F10      (VK_BASE + 121) //F10键
#define VK_F11      (VK_BASE + 122) //F11键
#define VK_F12      (VK_BASE + 124) //F12键
#define VK_NUMLOCK  (VK_BASE + 144) //Num Lock 键
#define VK_SCROLL   (VK_BASE + 145) //Scroll Lock键


#ifdef __cplusplus
}
#endif

#endif /* _KEY_CHAR_H_ */
