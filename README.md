# OS-Park: Real-Time Smart Parking System

A real-time smart parking management system on **STM32F411E-Discovery**
using **FreeRTOS**.
The system manages **3 parking slots** with real-time monitoring,
reservation, and occupancy tracking. Users interact with the system via
a potentiometer (slot selection) and a button (reserve/occupy). The
system provides visual feedback through an OLED display and LEDs, while
also logging events via SWV.

------------------------------------------------------------------------

## ✨ Features

-   🅿 **3 Parking Slots** with states:
    -   `EMPTY` → available
    -   `RESERVED` → held for 60s
    -   `OCCUPIED` → active usage (returns to RESERVED after 3s)
-   🎚 **Slot Selection** using potentiometer (via ADC)
-   🔘 **User Interaction** via button (EXTI interrupt)
-   📟 **Visual Feedback**:
    -   OLED display (I²C, SSD1306 driver)
    -   On-board LEDs indicating slot status
-   ⏱ **Real-Time Scheduling** with FreeRTOS tasks, queues, mutexes,
    event groups, and timers
-   🖥 **Debug Logging** through SWV (Serial Wire Viewer)

------------------------------------------------------------------------

## 🛠 Hardware

-   STM32F411E-Discovery development board
-   Potentiometer (slot selection, ADC input)
-   Push button (reservation/occupancy via EXTI)
-   SSD1306 OLED display (I²C)
-   On-board LEDs for status feedback

------------------------------------------------------------------------

## 💻 Software & FreeRTOS Components

-   **Tasks**
    -   `adcTask` → reads potentiometer, updates selected slot
    -   `buttonTask` → handles reservations and occupancy state machine
    -   `displayTask` → updates OLED with slot info and messages
    -   `eventMonitorTask` → logs system events via SWV
-   **Queues**
    -   `oledQueue` → messages for screen updates
    -   `commandQueue` → command handling
-   **Mutexes**
    -   `oledMutex` → protects I²C access to OLED
    -   `slotsMutex` → synchronizes access to slot state data
-   **Event Groups**
    -   `systemEventGroup` → tracks slot changes, reservations,
        occupancy, timeouts
-   **Software Timers**
    -   `vSlotTimerCb` → clears RESERVED/ OCCUPIED slots after 60s
    -   `vTempOccupiedTimerCb` → transitions OCCUPIED → RESERVED after
        3s

------------------------------------------------------------------------

## 📂 Project Structure

    ├── main.c                          # FreeRTOS-based embedded firmware
    ├── 05210000272_05210000290_...docx # Project report (detailed documentation)

------------------------------------------------------------------------

## ⚙️ Installation & Usage

1.  Open `main.c` in **STM32CubeIDE**.
2.  Flash firmware onto **STM32F411E-Discovery** board.
3.  Connect peripherals:
    -   Potentiometer to ADC input
    -   Button to EXTI pin (PA0)
    -   SSD1306 OLED display via I²C
4.  Observe real-time system behavior:
    -   Select slot with potentiometer
    -   Reserve/occupy slot using button
    -   Monitor updates on OLED and LEDs
    -   Debug logs available through **SWV Viewer**

------------------------------------------------------------------------

## 🎮 System Flow

1.  User selects a parking slot via potentiometer.
2.  User presses button:
    -   If slot = `EMPTY` → becomes `RESERVED` (60s timer starts).
    -   If slot = `RESERVED` → becomes `OCCUPIED` (3s timer, then back
        to `RESERVED`).
3.  Timers manage state transitions automatically.
4.  Display and LEDs update in real time.
5.  Event logs are printed via SWV.

------------------------------------------------------------------------
