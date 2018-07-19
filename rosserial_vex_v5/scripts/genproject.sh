# generate new pros project with name as first argument
prosv5 conductor new-project $1

# go into incude directory and generate ros message library.
cd $1/include
rosrun rosserial_vex_v5 make_libraries.py .

# unless the include is a special case
# (e.g. the include is a std library, or ros_lib is already in the path),
# add ros_lib to the start of the include.
grep -irl '#include' ros_lib/ | xargs perl -pi \
  -e 's/^#include "(?!main)(?!pros)(?!stddef)(?!stdlib)(?!string)(?!stdint)(?!math)(?!ros_lib)/#include "ros_lib\//g' \

# make directories for all the cpp files.

# move the cpp files into their proper places.
mv ros_lib/rosserial_vex_v5/examples/*.cpp ../src/
mv ros_lib/*.cpp ../src/
cd ..

# delete the incorrect files in src
rm src/*.c
