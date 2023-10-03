

# Reqirements

## GCC arm cross compiler
Since we are building for another processor than on our PC we need an arm cross compiler for c

on Ubuntu you can install
```
sudo apt install gcc-arm-none-eabi
```

## stm32cube library

To build the project you need the libraries required.
You can probably find them on the internet however the project was created with
```STM32CUBEMX```
so to get the libraries generate a new project for the ```STM32H755ZI-Q``` processor and the installation will ask you to also install these dependencies.


## Making
Now you can build the project

```
cd cube/makefile/
make
```
This will generete the .hex and .bin files

>Depending on where ```STM32CUBEMX``` installed the libraries there might be some linking issues with the makefile, so make sure to take a look at the paths if you cannot make

