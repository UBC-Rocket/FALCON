## Initial Setup
1. This is a zephyr application. To set up your Zephyr workspace, follow this tutorial: https://docs.zephyrproject.org/latest/develop/getting_started/index.html
2. Once your workspace is set up (i.e. zephyrproject, or whatever you named it), clone this repo into it.
3. Navigate into the FALCON repo: `cd falcon`
4. Build the applicaton (blinkytest, or sensortest, etc.), with your selected board from that applications '/boards' folder or a zephyr supported development board: (`west build blinkytest -b ubcrocket_fc_2526_r1 -p`
5. Connect the board to your computer and flash it: `west flash`
6. For a nice UI based developer experience, use VS Code with the 'Zephyr IDE Extension Pack' extension
