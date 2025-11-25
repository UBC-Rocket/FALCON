## Initial Setup
1. This is a zephyr module that contains an application. To set up your Zephyr workspace, follow this tutorial: https://docs.zephyrproject.org/latest/develop/getting_started/index.html
2. Once your workspace is set up (i.e. zephyrproject, or whatever you named it), cd into it and clone it: `cd zephyrproject`, `git clone https://...`
3. Navigate into the FALCON module/folder: `cd falcon`
4. Build the applicaton (blinkytest, or sensortest, etc.): </br>
   - Select a board, either a custom board from this repo's '/boards' folder (check the applications CMakeLists.txt for official board compatibility), or from Zephyrs official supported boards: https://zephyr-docs.listenai.com/boards/index.html
   - Build the application: `west build blinkytest -b ubcrocket_fc_2526_r1 -p`
6. Connect the board to your computer and flash it: `west flash`
7. For a nice UI based developer experience, use VS Code with the 'Zephyr IDE Extension Pack' extension
