#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import roslib; roslib.load_manifest("rosserial_arduino")
import rospy

import os, sys, subprocess

ros_types = {
    'bool'  :   ('bool',           1),
    'byte'  :   ('byte',           1),
    'char'  :   ('unsigned char',  1),
    'int8'  :   ('signed char',    1),
    'uint8' :   ('unsigned char',  1),
    'int16' :   ('int',            2),
    'uint16':   ('unsigned int',   2),
    'int32':    ('long',           4),
    'uint32':   ('unsigned long',  4),
    'float32':  ('float',          4)
}

def type_to_var(ty):
    lookup = {
        1 : 'unsigned char',
        2 : 'unsigned int',
        4 : 'unsigned long'
    }
    return lookup[ty]


#####################################################################
# Data Types

class EnumerationType:
    """ For data values. """
    
    def __init__(self, name, ty, value):
        self.name = name
        self.type = ty
        self.value = value
    
    def make_declaration(self, f):
        f.write('      enum { %s = %s };\n' % (self.name, str(self.value)))    

class PrimitiveDataType:
    """ Our datatype is a C/C++ primitive. """    

    def __init__(self, name, ty, bytes):
        self.name = name
        self.type = ty
        self.bytes = bytes

    def make_declaration(self, f):
        f.write('      %s %s;\n' % (self.type, self.name) )

    def serialize(self, f):
        #create a clean name
        cn = self.name.replace("[","").replace("]","").split(".")[-1]
        f.write('      union {\n')
        f.write('        %s real;\n' % self.type)
        f.write('        %s base;\n' % type_to_var(self.bytes))
        f.write('      } u_%s;\n' % cn)
        f.write('      u_%s.real = this->%s;\n' % (cn,self.name))
        for i in range(self.bytes):
            f.write('      *(outbuffer + offset + %d) = (u_%s.base >> (8 * %d)) & 0xFF;\n' % (i, cn, i) )
        f.write('      offset += sizeof(this->%s);\n' % self.name)

    def deserialize(self, f):
        cn = self.name.replace("[","").replace("]","").split(".")[-1]
        f.write('      union {\n')
        f.write('        %s real;\n' % self.type)
        f.write('        %s base;\n' % type_to_var(self.bytes))
        f.write('      } u_%s;\n' % cn)
        f.write('      u_%s.base = 0;\n' % cn)
        for i in range(self.bytes):
            f.write('      u_%s.base |= ((typeof(u_%s.base)) (*(inbuffer + offset + %d))) << (8 * %d);\n' % (cn,cn,i,i) )
        f.write('      this->%s = u_%s.real;\n' % (self.name, cn) )
        f.write('      offset += sizeof(this->%s);\n' % self.name)


class MessageDataType(PrimitiveDataType):
    """ For when our data type is another message. """
    def serialize(self, f):
        f.write('      offset += this->%s.serialize(outbuffer + offset);\n' % self.name)

    def deserialize(self, f):
        f.write('      offset += this->%s.deserialize(inbuffer + offset);\n' % self.name)


class Float64DataType(PrimitiveDataType):
    """ AVR C/C++ has no native 64-bit support, we automatically convert to 32-bit float. """
        
    def make_declaration(self, f):
        f.write('      float %s;\n' % self.name )
    
    def serialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      long * val_%s = (long *) &(this->%s);\n' % (cn,self.name))
        f.write('      long exp_%s = (((*val_%s)>>23)&255);\n' % (cn,cn))
        f.write('      if(exp_%s != 0)\n' % cn)
        f.write('        exp_%s += 1023-127;\n' % cn)
        f.write('      long sig_%s = *val_%s;\n' % (cn,cn))
        f.write('      *(outbuffer + offset++) = 0;\n') # 29 blank bits
        f.write('      *(outbuffer + offset++) = 0;\n')
        f.write('      *(outbuffer + offset++) = 0;\n')
        f.write('      *(outbuffer + offset++) = (sig_%s<<5) & 0xff;\n' % cn)
        f.write('      *(outbuffer + offset++) = (sig_%s>>3) & 0xff;\n' % cn)
        f.write('      *(outbuffer + offset++) = (sig_%s>>11) & 0xff;\n' % cn)
        f.write('      *(outbuffer + offset++) = ((exp_%s<<4) & 0xF0) | ((sig_%s>>19)&0x0F);\n' % (cn,cn))
        f.write('      *(outbuffer + offset++) = (exp_%s>>4) & 0x7F;\n' % cn)
        f.write('      if(this->%s < 0) *(outbuffer + offset -1) |= 0x80;\n' % self.name)

    def deserialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      unsigned long * val_%s = (unsigned long*) &(this->%s);\n' % (cn,self.name))
        f.write('      offset += 3;\n') # 29 blank bits
        f.write('      *val_%s = ((unsigned long)(*(inbuffer + offset++))>>5 & 0x07);\n' % cn)
        f.write('      *val_%s |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<3;\n' % cn)
        f.write('      *val_%s |= ((unsigned long)(*(inbuffer + offset++)) & 0xff)<<11;\n' % cn)
        f.write('      *val_%s |= ((unsigned long)(*(inbuffer + offset)) & 0x0f)<<19;\n' % cn)
        f.write('      unsigned long exp_%s = ((unsigned long)(*(inbuffer + offset++))&0xf0)>>4;\n' % cn)
        f.write('      exp_%s |= ((unsigned long)(*(inbuffer + offset)) & 0x7f)<<4;\n' % cn)
        f.write('      if(exp_%s !=0)\n' % cn)
        f.write('        *val_%s |= ((exp_%s)-1023+127)<<23;\n' % (cn,cn))
        f.write('      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->%s = -this->%s;\n' % (self.name,self.name))


class Int64DataType(PrimitiveDataType):
    """ AVR C/C++ has no native 64-bit support. """

    def make_declaration(self, f):
        f.write('      long %s;\n' % self.name )
    
    def serialize(self, f):
        for i in range(4):
            f.write('      *(outbuffer + offset++) = (%s >> (8 * %d)) & 0xFF;\n' % (self.name, i) )
        for i in range(4):
            f.write('      *(outbuffer + offset++) = (%s > 0) ? 0: 255;\n' % self.name )

    def deserialize(self, f):
        f.write('      %s = 0;\n' % self.name)
        for i in range(4):
            f.write('      %s += ((long)(*(inbuffer + offset++))) >> (8 * %d);\n' % (self.name, i))
        f.write('      offset += 4;\n')

    
class StringDataType(PrimitiveDataType):
    """ Need to convert to unsigned char *. """

    def make_declaration(self, f):
        f.write('      unsigned char * %s;\n' % self.name)

    def serialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      long * length_%s = (long *)(outbuffer + offset);\n' % cn)
        f.write('      *length_%s = strlen( (const char*) this->%s);\n' % (cn,self.name))
        f.write('      offset += 4;\n')
        f.write('      memcpy(outbuffer + offset, this->%s, *length_%s);\n' % (self.name,cn))
        f.write('      offset += *length_%s;\n' % cn)

    def deserialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      long * length_%s = (long *)(inbuffer + offset);\n' % cn)
        f.write('      offset += 4;\n')
        f.write('      this->%s = (inbuffer + offset);\n' % self.name)
        f.write('      offset += *length_%s;\n' % cn)


class TimeDataType(PrimitiveDataType):

    def __init__(self, name, ty, bytes):
        self.name = name
        self.type = ty
        self.sec = PrimitiveDataType(name+'.sec','unsigned long',4)
        self.nsec = PrimitiveDataType(name+'.nsec','unsigned long',4)

    def make_declaration(self, f):
        f.write('      %s %s;\n' % (self.type, self.name))

    def serialize(self, f):
        self.sec.serialize(f)
        self.nsec.serialize(f)

    def deserialize(self, f):
        self.sec.deserialize(f)
        self.nsec.deserialize(f)


class ArrayDataType(PrimitiveDataType):

    def __init__(self, name, ty, bytes, cls, array_size=None):
        self.name = name
        self.type = ty  
        self.bytes = bytes
        self.size = array_size
        self.cls = cls 

    def make_declaration(self, f):
        c = self.cls("*"+self.name, self.type, self.bytes)
        if self.size == None:
            f.write('      unsigned char %s_length;\n' % self.name)
            f.write('      %s st_%s;\n' % (self.type, self.name)) # static instance for copy
            f.write('      %s * %s;\n' % (self.type, self.name))
        else:
            f.write('      %s %s[%d];\n' % (self.type, self.name, self.size))
    
    def serialize(self, f):
        c = self.cls(self.name+"[i]", self.type, self.bytes)
        if self.size == None:
            # serialize length
            f.write('      *(outbuffer + offset++) = %s_length;\n' % self.name)
            f.write('      *(outbuffer + offset++) = 0;\n')
            f.write('      *(outbuffer + offset++) = 0;\n')
            f.write('      *(outbuffer + offset++) = 0;\n')
            f.write('      for( unsigned char i = 0; i < %s_length; i++){\n' % self.name)
            c.serialize(f)
            f.write('      }\n')
        else:
            f.write('      unsigned char * %s_val = (unsigned char *) this->%s;\n' % (self.name, self.name))    
            f.write('      for( unsigned char i = 0; i < %d; i++){\n' % (self.size) )
            c.serialize(f)            
            f.write('      }\n')
        
    def deserialize(self, f):
        if self.size == None:
            c = self.cls("st_"+self.name, self.type, self.bytes)
            # deserialize length
            f.write('      unsigned char %s_lengthT = *(inbuffer + offset++);\n' % self.name)
            f.write('      if(%s_lengthT > %s_length)\n' % (self.name, self.name))
            f.write('        this->%s = (%s*)realloc(this->%s, %s_lengthT * sizeof(%s));\n' % (self.name, self.type, self.name, self.name, self.type))
            f.write('      offset += 3;\n')
            f.write('      %s_length = %s_lengthT;\n' % (self.name, self.name))
            # copy to array
            f.write('      for( unsigned char i = 0; i < %s_length; i++){\n' % (self.name) )
            c.deserialize(f)
            f.write('        memcpy( &(this->%s[i]), &(this->st_%s), sizeof(%s));\n' % (self.name, self.name, self.type))                     
            f.write('      }\n')
        else:
            c = self.cls(self.name+"[i]", self.type, self.bytes)
            f.write('      unsigned char * %s_val = (unsigned char *) this->%s;\n' % (self.name, self.name))    
            f.write('      for( unsigned char i = 0; i < %d; i++){\n' % (self.size) )
            c.deserialize(f)            
            f.write('      }\n')


#####################################################################
# Messages

class Message:    
    """ Parses message definitions into something we can export. """

    def __init__(self, name, package, definition):

        self.name = name            # name of message/class
        self.package = package      # package we reside in
        self.includes = list()      # other files we must include

        self.data = list()          # data types for code generation
        self.enums = list()

        # parse definition
        for line in definition:
            # prep work
            line = line.strip().rstrip()    
            value = None
            if line.find("#") > -1:
                line = line[0:line.find("#")]
            if line.find("=") > -1:
                value = int(line[line.find("=")+1:])
                line = line[0:line.find("=")]
            
            # find package/class name   
            line = line.replace("\t", " ")
            l = line.split(" ")
            while "" in l:
                l.remove("")
            if len(l) < 2:
                continue
            ty, name = l[0:2]
            if value != None:
                self.enums.append( EnumerationType(name, ty, value))            
                continue

            try:
                type_package, type_name = ty.split("/")
            except:
                type_package = None
                type_name = ty
            type_array = False
            if type_name.find('[') > 0:
                type_array = True   
                try:
                    type_array_size = int(type_name[type_name.find('[')+1:type_name.find(']')])
                except:
                    type_array_size = None
                type_name = type_name[0:type_name.find('[')]

            # convert to C/Arduino type if primitive, expand name otherwise
            try:
                # primitive type
                cls = PrimitiveDataType
                code_type = type_name
                size = 0
                if type_package:
                    cls = MessageDataType                    
                if type_name == 'float64':
                    cls = Float64DataType   
                    code_type = 'float'
                elif type_name == 'time':
                    cls = TimeDataType
                    code_type = 'ros::Time'
                    if "ros/time" not in self.includes:
                        self.includes.append("ros/time")
                elif type_name == 'duration':
                    cls = TimeDataType
                    code_type = 'ros::Duration'
                    if "ros/duration" not in self.includes:
                        self.includes.append("ros/duration")
                elif type_name == 'string':
                    cls = StringDataType
                    code_type = 'unsigned char*'
                elif type_name == 'uint64' or type_name == 'int64':
                    cls = Int64DataType
                    code_type = 'long'
                else:
                    code_type = ros_types[type_name][0]
                    size = ros_types[type_name][1]
                if type_array:
                    self.data.append( ArrayDataType(name, code_type, size, cls, type_array_size ) )
                else:
                    self.data.append( cls(name, code_type, size) )
            except:
                if type_name == 'Header':
                    self.data.append( MessageDataType(name, 'std_msgs::Header', 0) )
                    if "std_msgs/Header" not in self.includes:
                        self.includes.append("std_msgs/Header")
                else:
                    if type_package == None or type_package == package:
                        type_package = package  
                        cls = MessageDataType
                        if self.package+"/"+type_name not in self.includes:
                            self.includes.append(self.package+"/"+type_name)
                    if type_package+"/"+type_name not in self.includes:
                        self.includes.append(type_package+"/"+type_name)
                    if type_array:
                        self.data.append( ArrayDataType(name, type_package + "::" + type_name, size, cls, type_array_size) )
                    else:
                        self.data.append( MessageDataType(name, type_package + "::" + type_name, 0) )
        #print ""

    def make_header(self, f):
        f.write('#ifndef ros_%s_h\n' % self.name)
        f.write('#define ros_%s_h\n' % self.name)
        f.write('\n')
        f.write('#include "WProgram.h"\n')
        f.write('#include "ros.h"\n')
        for i in self.includes:
            f.write('#include "%s.h"\n' % i)
        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')
        f.write('  class %s : public ros::Msg\n' % self.name)
        f.write('  {\n')
        f.write('    public:\n')

        for d in self.data:
            d.make_declaration(f)
        for e in self.enums:
            e.make_declaration(f)

        # serializer   
        f.write('\n')
        f.write('    virtual int serialize(unsigned char *outbuffer)\n')
        f.write('    {\n')
        f.write('      int offset = 0;\n')
        for d in self.data:
            d.serialize(f)
        f.write('      return offset;\n');
        f.write('    }\n')
        f.write('\n')

        # deserializer
        f.write('    virtual int deserialize(unsigned char *inbuffer)\n')
        f.write('    {\n')
        f.write('      int offset = 0;\n')
        for d in self.data:
            d.deserialize(f)
        f.write('     return offset;\n');
        f.write('    }\n')         
        f.write('\n')
        f.write('    const char * getType(){ return "%s/%s"; };\n'%(self.package, self.name))
        f.write('\n')
        f.write('  };\n')
        f.write('\n')
        f.write('}\n')

        f.write('#endif')

#####################################################################
# Core Library Maker

class ArduinoLibraryMaker:
    """ Create an Arduino Library from a set of Message Definitions. """

    def __init__(self, package):
        """ Initialize by finding location and all messages in this package. """
        self.name = package
        print "\nExporting " + package + ":", 

        # find directory for this package
        proc = subprocess.Popen(["rospack","find",msg_package], stdout=subprocess.PIPE)
        self.directory = proc.communicate()[0].rstrip() + "/msg"

        # find the messages in this package
        self.messages = list()
        for f in os.listdir(self.directory):
            if f.endswith(".msg"):
                # add to list of messages
                print "%s," % f[0:-4],
                definition = open(self.directory + "/" + f).readlines()
                self.messages.append( Message(f[0:-4], self.name, definition) )
        print "\n"
     

    def generate(self, path_to_output):
        """ Generate header and source files for this package. """

        # generate for each message
        for msg in self.messages:
            if not os.path.exists(path_to_output + "/" + self.name):
                os.makedirs(path_to_output + "/" + self.name)
            header = open(path_to_output + "/" + self.name + "/" + msg.name + ".h", "w")
            msg.make_header(header)
            header.close()

    
if __name__=="__main__":

    # get path to arduino sketchbook
    path = sys.argv[1]
    if path[-1] == "/":
        path = path[0:-1]
    #path += "/libraries"
    print "\nExporting to %s" % path

    # make libraries
    packages = sys.argv[2:]
    for msg_package in packages:
        lm = ArduinoLibraryMaker(msg_package)
        lm.generate(path)

