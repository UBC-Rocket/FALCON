## Initial Repo Setup
> Instructions to set up the FALCON workspace are designed for **Linux and MacOS**! Due to an upstream incompatibility with Windows and Zephyr, Windows users may face significant challenges. It is recommended to use [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) if you are on Windows.

FALCON uses Zephyr's workspace management and meta-tool, [west](https://docs.zephyrproject.org/latest/guides/west/index.html). As such, **do not clone this repo directly** (although it will still work with some additional manual setup). Instead, follow these steps:

1. (MacOS users only) Install Homebrew if you haven't already: `/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`
2. Install Python 3.10 or higher and Git if you haven't already.
3. Install a package runner for Python, such as `pipx` or `uvx`. uv is the official python package manager supported by UBCRocket, and is recommended. Follow there [instructions on the uv website](https://docs.astral.sh/uv/getting-started/installation/) to get started.
4. Clone the FALCON repository with west: `uvx west init -m git@github.com:UBC-Rocket/FALCON.git FALCON`
5. Navigate into the created folder and run `uvx west update`: `cd FALCON && uvx west update` (this might take a while)

## Development Setup
1. Install the required packages for Zephyr development:
   - Follow the `Install dependencies` instructions here (don't follow the rest of the instructions): https://docs.zephyrproject.org/latest/develop/getting_started/index.html#installing-prerequisites
   - Make sure to install `cmake`, `ninja`, `dtc`, `gcc-arm-none-eabi` (or other toolchain depending on your board), `openocd`, and `pyocd`
2. Navigate into the FALCON folder if not already: `cd FALCON`
3. Create and activate a python virtual environment:
   - `uv venv --seed` (The `seed` flag is important!)
   - `source .venv/bin/activate` (MacOS/Linux) or `.venv\Scripts\activate.bat` (Windows)
4. Install `west` in the virtual environment: `pip install west`
5. Installed the required Python packages for Zephyr: `west packages pip --install` (Note that we are **no longer** using `uvx west`)
6. Install the Zephyr SDK: `west sdk install`
7. Build the application (blinkytest, or sensortest, etc.):  
   - Select a board, either a custom board from this repo's '/boards' folder (check the applications CMakeLists.txt for official board compatibility), or from Zephyrs official supported boards: https://zephyr-docs.listenai.com/boards/index.html
   - Build the application: `west build blinkytest -b ubcrocket_polarity -p`
8. Connect the board to your computer and flash it: `west flash`

For a nice UI based developer experience, use VS Code with the 'Zephyr IDE Extension Pack' extension

## Zephyr VSCode Setup
TODO

## Knowledge
- Zephyr Documentation: https://docs.zephyrproject.org/latest/index.html
- Highly recommend watching this tutorial series (Videos 9, 10, and 11 shouldnt be necessary for this project): https://www.youtube.com/playlist?list=PLEBQazB0HUyTmK2zdwhaf8bLwuEaDH-52

## Integration Testing
### Zephyr's Native Sim
#### Getting the data
1. Follow the installation instruction for OpenRocket: https://openrocket.readthedocs.io/en/latest/setup/installation.html 
2. Boot up OpenRocket and load the .ork file
3. Under Flight Simulations, open a previous simulation by double clicking on the row
4. Under Export Data, click "select all" and ensure that only the "Include field descriptions" option is checked.
5. Click Export and save your csv file

#### Running the integration test
1. Ensure you're in the virtual environment using inside `FALCON/`:
   - `source .venv/bin/activate` (MacOS/Linux) or `.venv\Scripts\activate.bat` (Windows)
2. Build FALCON using Zephyr's built in native_sim board:
   - `west build -b native_sim/native/64 firmware -- -DDTC_OVERLAY_FILE="$(pwd)/boards/native_sim.overlay" -DDATA_FILE="$(pwd)/<PATH>/<DATA_FILE>.csv"`
   - Replace `PATH` and `DATA_FILE` with the location and name of your OpenRocket csv file
3. Run the integration test:
   - For a realtime test, use `west build -t run`
   - To run at the maximum speed, use `./app/build/zephyr/zephyr.exe --no-rt`
   - To run at a custom speed, use `./app/build/zephyr/zephyr.exe --rt-ratio=2`

### QEMU (WIP)


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
