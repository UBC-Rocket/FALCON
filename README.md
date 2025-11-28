## Initial Setup
1. This is a zephyr module that contains an application. To set up your Zephyr workspace, follow this tutorial: https://docs.zephyrproject.org/latest/develop/getting_started/index.html
2. Once your workspace is set up (i.e. zephyrproject, or whatever you named it), cd into it and clone it: `cd zephyrproject`, `git clone https://...`
3. Activate the venv if not already: macOS `source .venv/bin/activate`, windows depends on your shell, look it up.
   - You will active the venv every time to build from the CLI
4. Navigate into the FALCON module/folder: `cd falcon`
5. . Build the applicaton (blinkytest, or sensortest, etc.): </br>
   - Select a board, either a custom board from this repo's '/boards' folder (check the applications CMakeLists.txt for official board compatibility), or from Zephyrs official supported boards: https://zephyr-docs.listenai.com/boards/index.html
   - Build the application: `west build blinkytest -b ubcrocket_fc_2526_r1 -p`
6. Connect the board to your computer and flash it: `west flash`
7. For a nice UI based developer experience, use VS Code with the 'Zephyr IDE Extension Pack' extension

## Knowledge
- Zephyr Documentation: https://docs.zephyrproject.org/latest/index.html
- Highly recommend watching this tutorial series (Videos 9, 10, and 11 shouldnt be necessary for this project): https://www.youtube.com/playlist?list=PLEBQazB0HUyTmK2zdwhaf8bLwuEaDH-52

## Debugging
### Terminal debugging:
*Should work out of the box provided your zephyr environment is set up correctly*
- It's important that you set the `CONFIG_NO_OPTIMIZATIONS=y` flag to 'y' in the prj.conf file so that it does not optimize your code and move around the line numbers.
- cd into `FALCON`, and run `west build ...`
- run `west debug`
- This should launch a gdb session from which you can debug (painfully)
### VS Code Debugging (with Zephyr IDE extension)
- Install the Cortex-Debug extension
- in the .vscode folder in your root workspace folder (i.e. zephyrproject, or whatever you named it) create a launch.json file and paste the following:
   ```
   {
       // Use IntelliSense to learn about possible attributes.
       // Hover to view descriptions of existing attributes.
       // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
       "version": "0.2.0",
       "configurations": [
           {
               "name": "Zephyr IDE: Debug",
               "cwd": "${workspaceFolder}",
               "gdbPath": "${command:zephyr-ide.get-gdb-path}",
               "executable": "${command:zephyr-ide.get-active-build-path}/zephyr/zephyr.elf",
               "request": "launch",
               "type": "cortex-debug",
               "servertype": "pyocd",
               "serverArgs": ["-t", "STM32H563vitx"],
               "interface": "swd",
               "serverpath": "${workspaceFolder}/.venv/bin/pyocd",
           }
       ]
   }
   ```
- In the root workspace folder create a `pyocd.yaml` file and paste the following:
  ```
  pack:
     - ./FALCON/packs/Keil.STM32H5xx_DFP.2.1.1.pack
  ```
- From Zephyr IDE create a build for your project with 'No Optimizations' (Debug option does not work for some reason)
- Select that build and press on `Build and Debug`. This should launch the VS Code Debugger
