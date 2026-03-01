# esc-bootloader

## Сборка проекта

### 1. Установка тулчейна

Перейдите в папку `tools` и запустите скрипт установки тулчейна в зависимости от целевой архитектуры:

- **RISC-V** (`ch32v`): `dependencies/tools/risc/install-toolchain.cmd`
- **ARM** (`stm32` и др.): `dependencies/tools/arm/install-toolchain.cmd` *(папка появится позже)*

### 2. Установка OpenOCD

Используется патченая версия OpenOCD из состава **MounRiver Studio**.

**Шаги:**

1. Скачайте и установите [MounRiver Studio](http://www.mounriver.com/download)
2. После установки запустите скрипт копирования OpenOCD:
   ```
   dependencies/tools/openocd/copy-openocd-mounriver.cmd
   ```
   Скрипт скопирует патченый OpenOCD из директории установки MounRiver Studio в нужное место.

### 3. Сборка

После установки тулчейна и OpenOCD проект собирается через CMake стандартным образом.