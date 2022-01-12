# generate new pros project with name as first argument
pros conduct new $1

# go into incude directory and generate ros message library.
cd $1/include
rosrun rosserial_vex_cortex make_libraries.py .

# custom include for files containing strlen.
# grep -lnrw '.' -e 'strlen'
grep -lrnw '.' -e 'strlen' | xargs sed -i '1 s@^@#include "vexstrlen.h"\n@g; s@strlen(@vexstrlen(@g'                                        

# move the cpp files into the src directory.
mv ros_lib/*.cpp ../src
mv ros_lib/examples/*.cpp ../src
cd ..

# delete the incorrect files in src
rm src/*.c

# replace the common.mk with the correct one.
sed -i '/^INCLUDE=/ s@$@ -I$(ROOT)/include/ros_lib@' common.mk
