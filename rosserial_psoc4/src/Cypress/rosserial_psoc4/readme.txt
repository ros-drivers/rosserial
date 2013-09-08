This directory holds the output files produced by Cypress PSoC Creator, a Windows-only program

1. Open the rosserial-psoc4 workspace in Creator. It contains several example projects
2. Select a project (e.g. HelloWorld) and build it. This will produce
   a) a set of .c and .h source files in HelloWorld.cydsn/Generated_Source
   b) HelloWorld.cydsn/device.h
   c) many other files that we do not use
3. Copy the HelloWorld.cydsn directory from the Windows machine to this directory. It is only
   *necessary* to copy a) and b) files above, but moving the whole directory is ok.

Note: PSoC Creator can leave abandoned files in the Generated_Source directory which do not show
  up in the Creator IDE but *are* on the disk and can interfere with the linux build (we try to
  build *all* the source files in the directory). This will occur if you remove or change the
  name of a PSoC component in your design. The most reliable "clean" is to delete the entire
  Generated_Source directory before rebuilding the project in Creator.


