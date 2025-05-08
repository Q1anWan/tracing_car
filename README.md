## 迁移到新平台后如何重新在CLion中开启工程：
1. Middlewares, cmake, .ld, .idea文件与文件夹已经被忽略, 因此首先使用CubeMX Generate Code
2. .ld文件与cmake关联, 第一次重新生成cmake文件后需要覆写.ld文件. 使用backup.ld覆盖浅层目录下STM32*_FLASH.ld
3. 使用CLion重新添加STM32工程

## 包版本
1. STM32CubeH7: 1.12.1
2. AZRTOS-H7: 3.3.0

## 工具版本
1. STM32CubeMX: 6.14.1
2. CLion: 25.1.1
3. CubeCLT: 1.18.0