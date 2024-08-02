# Getting Started (Linux)

This guide describes how to run the Open1722 example applications on Linux.

## System Requirements

Open1722 depends on the following software components:

- CMake >= 3.20
- CMocka >= 1.1.0

## Build Open1722 from Source

To build Open1722 from sources run the following commands:

``` txt
$ mkdir build
$ cd buid
$ cmake ..
$ make
```

The Open1722 libraries and applications can be optionally be installed to the system: 

``` txt
$ sudo make install
```

For a clean build just delete the build directory and start all over again:

``` txt
$ rm -r build
```

## Running Sample Applications

## Running Tests
