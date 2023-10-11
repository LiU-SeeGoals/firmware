
# How to use

## Adding files

When adding .c files to the project make sure they are added to MakeFile for /MakeFile/CM7/MakFile or /MakeFile/CM4/MakeFile (CM7 is default)

# Reqirements

## GCC arm cross compiler
Since the devboard cpu has different architecture (arm?) we need a cross compiler (your default gcc compiler will most likely not work)

on Ubuntu you can install
```
sudo apt install gcc-arm-none-eabi
```

## stm32cube library

To build the project you need the libraries required.
You can probably find them on the internet however the project was created with
```STM32CUBEMX```

Open the .ioc file with ```STM32CUBEMX``` and press generate code this will then prompt to install the required dependencies (STMCUBE32) 

> If this does not work for some reason you can try

So to get the libraries generate a new project for the ```STM32H755ZI-Q``` processor and the installation will ask you to also install these dependencies.

## Making

Now you can build the project

```
cd cube/toebite/makefile/
make
```

This will generete the .hex and .bin files

>Depending on where ```STM32CUBEMX``` installed the libraries there might be some linking issues with the makefile, so make sure to take a look at the paths in the MakeFiles if it does not work

## programming 

There are two option one for command line and one magic gui (that works)

### Terminal based
```
sudo apt install stlink-tools
```

Now from 
```
cd cube/toebite/Makefile
make && st-flash --reset write CM7/build/toebite_CM7.bin 0x08000000
```

### GUI

Or install the GUI program stmcube32prog which is easier to use.

Click Erasing & programming choose the .bin file create from make process as file path.

Enable Run after programming

Connect to board at the top right corner and click start programming


# Syntax checking and Intellisense

Anton uses these settings with vscode

You can probably reverse engineer it 

Compile_commands.json is also important. To create this do

```
cd cube/toebite/Makefile/
bear -- make
```

and move the file to the correct path in "compileCommands"

```
// Name ${workspaceRoot} folder path names to your project
// Set paths to arm-none-eabi libraries as it is installed on your system
{
    "configurations": [
        {
            "name": "Linux",
            "compileCommands": "${workspaceRoot}/.vscode/compile_commands.json",
            "includePath":
            [
                "${workspaceRoot}",
                "${workspaceRoot}/cube/toebite/CM7/Core/Inc",
                "${workspaceRoot}/cube/toebite/Drivers/STM32H7xx_HAL_Driver/Inc",
                "${workspaceRoot}/cube/toebite//Drivers/CMSIS/Include",
                "${workspaceRoot}/cube/toebite/Drivers/CMSIS/Device/ST/STM32H7xx/Include",
                "${workspaceRoot}/cube/toebite/Drivers/CMSIS/Device/ST/STM32H7xx/Include",
                "/usr/lib/arm-none-eabi/include",
                "/usr/lib/arm-none-eabi/include/c++/10.3.1",
                "/lib/gcc/arm-none-eabi/10.3.1/include",
                "/lib/gcc/arm-none-eabi/10.3.1/include-fixed",
                "/usr/lib/arm-none-eabi/include/machine",
                "/usr/lib/arm-none-eabi/include/nano",
                "/usr/lib/arm-none-eabi/include/sys"
            ],
            "defines":
            [
                "DCORE_CM7",
                "DUSE_HAL_DRIVER",
                "DSTM32H755xx"
            ],
            "intelliSenseMode": "linux-gcc-x64",
            "cStandard": "c11",
            "cppStandard": "c++20",
            "browse":
            {
                "path":
                [
                    "${workspaceRoot}",
                    "${workspaceRoot}/cube/toebite/CM7/Core/Inc",
                    "${workspaceRoot}/cube/toebite/Drivers/STM32H7xx_HAL_Driver/Inc",
                    "${workspaceRoot}/cube/toebite/Drivers/CMSIS/Include",
                    "${workspaceRoot}/cube/toebite/Drivers/CMSIS/Device/ST/STM32H7xx/Include",
                    "/usr/lib/arm-none-eabi/include",
                    "/usr/lib/arm-none-eabi/include/c++/10.3.1",
                    "/lib/gcc/arm-none-eabi/10.3.1/include",
                    "/lib/gcc/arm-none-eabi/10.3.1/include-fixed"
                ],
                "limitSymbolsToIncludedHeaders": true,
                "databaseFilename": "${workspaceRoot}/.vscode/browse.vc.db"
            }
        }
    ],
    "version": 4
}
```

# Tutorials

> Programming the MCU: https://www.youtube.com/watch?v=1cleO3mHjWw&list=PLEg2mgYz66IOcHRvvUDf9O1ZCGy58M1Bt&index=3

> Creating/Building project with make and STM32cubeMX https://www.youtube.com/watch?v=FkqQpBqkSns&list=PLEg2mgYz66IOcHRvvUDf9O1ZCGy58M1Bt&index=1
