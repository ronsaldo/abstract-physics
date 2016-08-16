# AbstractPhysics
The Abstract Physics is a cross platform abstraction layer above physics engine
such as Bullet. The objetive of this abstraction layer is to export a simple to
use C interface that can be called via FFI from some dynamic languages such as
Pharo

# Building instruction for Linux and Max OS/X
For building the abstraction layer for Linux and Max OS/X, CMake is required.
The following commands can be used for building:

    mkdir build
    cd build
    cmake ..
    make

The built files will be available at the dist folder. The samples will not be
built by default.
