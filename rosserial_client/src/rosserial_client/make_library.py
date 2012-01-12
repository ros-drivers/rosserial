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

__usage__ = """
make_library.py generates the Arduino rosserial library files.  It 
requires the location of your arduino libraries folder and the name of 
one or more packages for which you want to make libraries.  

rosrun rosserial_client make_library.py <output_path> pkg_name [pkg2 pkg3 ...]
"""

import roslib; roslib.load_manifest("rosserial_client")
import roslib.gentools, roslib.srvs
import roslib.rospack
import rospy

import os, sys, subprocess, re

def type_to_var(ty):
    lookup = {
        1 : 'uint8_t',
        2 : 'uint16_t',
        4 : 'uint32_t',
        8 : 'uint64_t',
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
        f.write('      enum { %s = %s };\n' % (self.name, self.value))    

class PrimitiveDataType:
    """ Our datatype is a C/C++ primitive. """    

    def __init__(self, name, ty, bytes):
        self.name = name
        self.type = ty
        self.bytes = bytes

    def make_declaration(self, f):
        f.write('      %s %s;\n' % (self.type, self.name) )

    def serialize(self, f):
        cn = self.name.replace("[","").replace("]","").split(".")[-1]
        if self.type != type_to_var(self.bytes):
            f.write('      union {\n')
            f.write('        %s real;\n' % self.type)
            f.write('        %s base;\n' % type_to_var(self.bytes))
            f.write('      } u_%s;\n' % cn)
            f.write('      u_%s.real = this->%s;\n' % (cn,self.name))
            for i in range(self.bytes):
                f.write('      *(outbuffer + offset + %d) = (u_%s.base >> (8 * %d)) & 0xFF;\n' % (i, cn, i) )
        else:
            for i in range(self.bytes):
                f.write('      *(outbuffer + offset + %d) = (this->%s >> (8 * %d)) & 0xFF;\n' % (i, self.name, i) )
        f.write('      offset += sizeof(this->%s);\n' % self.name)

    def deserialize(self, f):
        cn = self.name.replace("[","").replace("]","").split(".")[-1]
        if self.type != type_to_var(self.bytes):
            f.write('      union {\n')
            f.write('        %s real;\n' % self.type)
            f.write('        %s base;\n' % type_to_var(self.bytes))
            f.write('      } u_%s;\n' % cn)
            f.write('      u_%s.base = 0;\n' % cn)
            for i in range(self.bytes):
                f.write('      u_%s.base |= ((%s) (*(inbuffer + offset + %d))) << (8 * %d);\n' % (cn,type_to_var(self.bytes),i,i) )
            f.write('      this->%s = u_%s.real;\n' % (self.name, cn) )
        else:
            f.write('      this->%s =  ((%s) (*(inbuffer + offset)));\n' % (self.name,self.type) )
            for i in range(self.bytes-1):
                f.write('      this->%s |= ((%s) (*(inbuffer + offset + %d))) << (8 * %d);\n' % (self.name,self.type,i+1,i+1) )
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
        f.write('      int32_t * val_%s = (long *) &(this->%s);\n' % (cn,self.name))
        f.write('      int32_t exp_%s = (((*val_%s)>>23)&255);\n' % (cn,cn))
        f.write('      if(exp_%s != 0)\n' % cn)
        f.write('        exp_%s += 1023-127;\n' % cn)
        f.write('      int32_t sig_%s = *val_%s;\n' % (cn,cn))
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
        f.write('      uint32_t * val_%s = (uint32_t*) &(this->%s);\n' % (cn,self.name))
        f.write('      offset += 3;\n') # 29 blank bits
        f.write('      *val_%s = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);\n' % cn)
        f.write('      *val_%s |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;\n' % cn)
        f.write('      *val_%s |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;\n' % cn)
        f.write('      *val_%s |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;\n' % cn)
        f.write('      uint32_t exp_%s = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;\n' % cn)
        f.write('      exp_%s |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;\n' % cn)
        f.write('      if(exp_%s !=0)\n' % cn)
        f.write('        *val_%s |= ((exp_%s)-1023+127)<<23;\n' % (cn,cn))
        f.write('      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->%s = -this->%s;\n' % (self.name,self.name))

    
class StringDataType(PrimitiveDataType):
    """ Need to convert to signed char *. """

    def make_declaration(self, f):
        f.write('      char * %s;\n' % self.name)

    def serialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      uint32_t * length_%s = (uint32_t *)(outbuffer + offset);\n' % cn)
        f.write('      *length_%s = strlen( (const char*) this->%s);\n' % (cn,self.name))
        f.write('      offset += 4;\n')
        f.write('      memcpy(outbuffer + offset, this->%s, *length_%s);\n' % (self.name,cn))
        f.write('      offset += *length_%s;\n' % cn)

    def deserialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      uint32_t length_%s = *(uint32_t *)(inbuffer + offset);\n' % cn)
        f.write('      offset += 4;\n')
        f.write('      for(unsigned int k= offset; k< offset+length_%s; ++k){\n'%cn) #shift for null character
        f.write('          inbuffer[k-1]=inbuffer[k];\n')
        f.write('      }\n')
        f.write('      inbuffer[offset+length_%s-1]=0;\n'%cn)
        f.write('      this->%s = (char *)(inbuffer + offset-1);\n' % self.name)
        f.write('      offset += length_%s;\n' % cn)


class TimeDataType(PrimitiveDataType):

    def __init__(self, name, ty, bytes):
        self.name = name
        self.type = ty
        self.sec = PrimitiveDataType(name+'.sec','uint32_t',4)
        self.nsec = PrimitiveDataType(name+'.nsec','uint32_t',4)

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
            f.write('      uint8_t %s_length;\n' % self.name)
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
            f.write('      for( uint8_t i = 0; i < %s_length; i++){\n' % self.name)
            c.serialize(f)
            f.write('      }\n')
        else:
            f.write('      unsigned char * %s_val = (unsigned char *) this->%s;\n' % (self.name, self.name))    
            f.write('      for( uint8_t i = 0; i < %d; i++){\n' % (self.size) )
            c.serialize(f)            
            f.write('      }\n')
        
    def deserialize(self, f):
        if self.size == None:
            c = self.cls("st_"+self.name, self.type, self.bytes)
            # deserialize length
            f.write('      uint8_t %s_lengthT = *(inbuffer + offset++);\n' % self.name)
            f.write('      if(%s_lengthT > %s_length)\n' % (self.name, self.name))
            f.write('        this->%s = (%s*)realloc(this->%s, %s_lengthT * sizeof(%s));\n' % (self.name, self.type, self.name, self.name, self.type))
            f.write('      offset += 3;\n')
            f.write('      %s_length = %s_lengthT;\n' % (self.name, self.name))
            # copy to array
            f.write('      for( uint8_t i = 0; i < %s_length; i++){\n' % (self.name) )
            c.deserialize(f)
            f.write('        memcpy( &(this->%s[i]), &(this->st_%s), sizeof(%s));\n' % (self.name, self.name, self.type))                     
            f.write('      }\n')
        else:
            c = self.cls(self.name+"[i]", self.type, self.bytes)
            f.write('      uint8_t * %s_val = (uint8_t*) this->%s;\n' % (self.name, self.name))    
            f.write('      for( uint8_t i = 0; i < %d; i++){\n' % (self.size) )
            c.deserialize(f)            
            f.write('      }\n')


ros_to_arduino_types = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           4, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          4, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('float',             4, Float64DataType, []),
    'time'    :   ('ros::Time',         8, TimeDataType, ['ros/time']),
    'duration':   ('ros::Duration',     8, TimeDataType, ['ros/duration']),
    'string'  :   ('char*',             0, StringDataType, []),
    'Header'  :   ('std_msgs::Header',  0, MessageDataType, ['std_msgs/Header'])
}


#####################################################################
# Messages

class Message:    
    """ Parses message definitions into something we can export. """

    def __init__(self, name, package, definition, md5):

        self.name = name            # name of message/class
        self.package = package      # package we reside in
        self.md5 = md5              # checksum
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
                try:
                    value = line[line.find("=")+1:]
                except:
                    value = '"' + line[line.find("=")+1:] + '"';
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

            # convert to C type if primitive, expand name otherwise
            try:
                code_type = ros_to_arduino_types[type_name][0]
                size = ros_to_arduino_types[type_name][1]
                cls = ros_to_arduino_types[type_name][2]
                for include in ros_to_arduino_types[type_name][3]:
                    if include not in self.includes:
                        self.includes.append(include)
            except:
                if type_package == None:
                    type_package = self.package
                if type_package+"/"+type_name not in self.includes:
                    self.includes.append(type_package+"/"+type_name)
                cls = MessageDataType
                code_type = type_package + "::" + type_name
                size = 0
            if type_array:
                self.data.append( ArrayDataType(name, code_type, size, cls, type_array_size ) )
            else:
                self.data.append( cls(name, code_type, size) )

    def _write_serializer(self, f):
                # serializer   
        f.write('\n')
        f.write('    virtual int serialize(unsigned char *outbuffer) const\n')
        f.write('    {\n')
        f.write('      int offset = 0;\n')
        for d in self.data:
            d.serialize(f)
        f.write('      return offset;\n');
        f.write('    }\n')
        f.write('\n')
        
    def _write_deserializer(self, f):
        # deserializer
        f.write('    virtual int deserialize(unsigned char *inbuffer)\n')
        f.write('    {\n')
        f.write('      int offset = 0;\n')
        for d in self.data:
            d.deserialize(f)
        f.write('     return offset;\n');
        f.write('    }\n')         
        f.write('\n')

    def _write_std_includes(self, f):
        f.write('#include <stdint.h>\n')
        f.write('#include <string.h>\n')
        f.write('#include <stdlib.h>\n')
        f.write('#include "ros/msg.h"\n')

    def _write_msg_includes(self,f):
        for i in self.includes:
            f.write('#include "%s.h"\n' % i)
            
    def _write_data(self, f):
        for d in self.data:
            d.make_declaration(f)
        for e in self.enums:
            e.make_declaration(f)
            
    def _write_getType(self, f):
        f.write('    const char * getType(){ return "%s/%s"; };\n'%(self.package, self.name))

    def _write_getMD5(self, f):
        f.write('    const char * getMD5(){ return "%s"; };\n'%self.md5)

    def _write_impl(self, f):
        f.write('  class %s : public ros::Msg\n' % self.name)
        f.write('  {\n')
        f.write('    public:\n')
        self._write_data(f)
        self._write_serializer(f)
        self._write_deserializer(f)
        self._write_getType(f)
        self._write_getMD5(f)
        f.write('\n')
        f.write('  };\n')
        
    def make_header(self, f):
        f.write('#ifndef _ROS_%s_%s_h\n'%(self.package, self.name))
        f.write('#define _ROS_%s_%s_h\n'%(self.package, self.name))
        f.write('\n')
        self._write_std_includes(f)
        self._write_msg_includes(f)
       
        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')
        self._write_impl(f)
        f.write('\n')
        f.write('}\n')

        f.write('#endif')

class Service:
    def __init__(self, name, package, definition, md5req, md5res):
        """ 
        @param name -  name of service
        @param package - name of service package
        @param definition - list of lines of  definition
        """
        
        self.name = name
        self.package = package
        
        sep_line = None
        sep = re.compile('---*')
        for i in range(0, len(definition)):
            if (None!= re.match(sep, definition[i]) ):
                sep_line = i
                break
        self.req_def = definition[0:sep_line]
        self.resp_def = definition[sep_line+1:]
        
        self.req = Message(name+"Request", package, self.req_def, md5req)
        self.resp = Message(name+"Response", package, self.resp_def, md5res)
        
    def make_header(self, f):
        f.write('#ifndef _ROS_SERVICE_%s_h\n' % self.name)
        f.write('#define _ROS_SERVICE_%s_h\n' % self.name)
        
        self.req._write_std_includes(f)
        includes = self.req.includes
        includes.extend(self.resp.includes)
        includes = list(set(includes))
        for inc in includes:
            f.write('#include "%s.h"\n' % inc)
            
        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')       
        f.write('static const char %s[] = "%s/%s";\n'%(self.name.upper(), self.package, self.name))
        
        def write_type(out, name):
            out.write('    const char * getType(){ return %s; };\n'%(name))
        _write_getType = lambda out: write_type(out, self.name.upper())
        self.req._write_getType = _write_getType
        self.resp._write_getType = _write_getType
        
        f.write('\n')
        self.req._write_impl(f)
        f.write('\n')
        self.resp._write_impl(f)
        f.write('\n')
        f.write('  class %s {\n' % self.name )
        f.write('    public:\n')
        f.write('    typedef %s Request;\n' % self.req.name )
        f.write('    typedef %s Response;\n' % self.resp.name )
        f.write('  };\n')
        f.write('\n')

        f.write('}\n')

        f.write('#endif')
        
   
        
#####################################################################
# Make a Library

def MakeLibrary(package, output_path):
    print "Exporting " + package + "\n", 

    pkg_dir = roslib.packages.get_pkg_dir(package)
        
    sys.stdout.write('  Messages:')
    # find the messages in this package
    messages = list()
    if os.path.exists(pkg_dir+"/msg"):
        sys.stdout.write('\n    ')
        for f in os.listdir(pkg_dir+"/msg"):
            if f.endswith(".msg"):
                file = pkg_dir + "/msg/" + f
                # add to list of messages
                print "%s," % f[0:-4],
                definition = open(file).readlines()
                md5sum = roslib.gentools.compute_md5(roslib.gentools.get_file_dependencies(file)) 
                messages.append( Message(f[0:-4], package, definition, md5sum) )
    print "\n"
     
    sys.stdout.write('  Services:')
    # find the services in this package
    services = list()
    if (os.path.exists(pkg_dir+"/srv/")):
        sys.stdout.write('\n    ')
        for f in os.listdir(pkg_dir+"/srv"):
            if f.endswith(".srv"):
                file = pkg_dir + "/srv/" + f
                # add to list of messages
                print "%s," % f[0:-4],
                definition, service = roslib.srvs.load_from_file(file)
                definition = open(file).readlines()
                md5req = roslib.gentools.compute_md5(roslib.gentools.get_dependencies(service.request, package))
                md5res = roslib.gentools.compute_md5(roslib.gentools.get_dependencies(service.response, package))
                messages.append( Service(f[0:-4], package, definition, md5req, md5res ) )
    print "\n"
    
    # generate for each message
    output_path = output_path + "/" + package
    for msg in messages:
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        header = open(output_path + "/" + msg.name + ".h", "w")
        msg.make_header(header)
        header.close()


def add_depends(packages, package):
    depend = [package] + roslib.rospack.rospack_depends(package)
    for p in depend:
        if not p in packages:
            packages.append(p)
            packages = add_depends(packages, p)
    return packages
    
if __name__=="__main__":
    
    # need correct inputs
    if (len(sys.argv) <3):
        print __usage__
        exit()
    
    # get output path
    path = sys.argv[1]
    if path[-1] == "/":
        path = path[0:-1]
    path += "/ros_lib"
    print "\nExporting to %s" % path

    packages = list()
    # make libraries
    for package in sys.argv[2:]:
        packages = add_depends(packages, package)

    print packages
    for package in packages:
        MakeLibrary(package, path)

