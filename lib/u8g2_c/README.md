# u8g2 核心（C 版）+ MUI 表单库 —— vendor 自 Arduino 包

**来源**：PlatformIO registry `olikraus/U8g2@2.36.18` 的 `src/clib/`。
**为什么 vendor 到这里**：本工程是 `framework = stm32cube`，而那个包是 **Arduino 结构**
（源码在 `src/clib/` 子目录、顶层是 `U8g2lib.cpp` 等 C++ 包装）。PlatformIO 只把包的
`src/` 顶层加进 include path，子目录不会被收录 —— 直接把包当 `lib_deps` 用会报
`u8g2.h: No such file or directory`。放进 PIO 的 `lib/<名字>/` 才会被自动编译并加头文件路径。

**只用了 C 那部分**：`u8g2.c` 系列 + `mui.c` + `mui_u8g2.c`。Arduino 的 C++ 包装
（`U8g2lib.*` / `U8x8lib.*` / `MUIU8g2.h`）没有拿过来，本工程是纯 C。

**实测占用**（STM32F103VET6，SSD1306 I2C 128x64，探针强制保留符号）：
Flash **+2.4 KB**、RAM **+1.3 KB**（不含字体；英文小字体再加约 1~2 KB）。

**升级方式**：`pio pkg install --library "olikraus/U8g2@<版本>"`，再从
`.pio/libdeps/genericSTM32F103VE/U8g2/src/clib/` 复制 `.c/.h` 过来。
**不要**从 GitHub 拉 —— 本环境 github.com 被 DNS 拦。

**上游文档**：<https://github.com/olikraus/u8g2/wiki>（MUI 手册见 `muimanual` 一节；
示例在包的 `examples/mui/`，其中 `MUIInput3BtnWithU8g2` 就是 3 按键输入）。
