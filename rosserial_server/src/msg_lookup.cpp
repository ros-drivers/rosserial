/**
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2019, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */

#include "rosserial_server/msg_lookup.h"
#include "Python.h"

namespace rosserial_server
{

const MsgInfo lookupMessage(const std::string& message_type, const std::string submodule)
{
  // Lazy-initialize the embedded Python interpreter. Avoid calling the corresponding
  // finalize method due to issues with importing cyaml in the second instance. The
  // total memory cost of having this in-process is only about 5-6MB.
  Py_Initialize();

  MsgInfo msginfo;
  size_t slash_pos = message_type.find('/');
  if (slash_pos == std::string::npos)
  {
    throw std::runtime_error("Passed message type string does not include a slash character.");
  }
  std::string module_name = message_type.substr(0, slash_pos);
  std::string class_name = message_type.substr(slash_pos + 1, std::string::npos);

  PyObject* module = PyImport_ImportModule((module_name + "." + submodule).c_str());
  if (!module)
  {
    throw std::runtime_error("Unable to import message module " + module_name + ".");
  }
  PyObject* msg_class = PyObject_GetAttrString(module, class_name.c_str());
  if (!msg_class)
  {
    throw std::runtime_error("Unable to find message class " + class_name +
                             " in module " + module_name + ".");
  }
  Py_XDECREF(module);

  PyObject* full_text = PyObject_GetAttrString(msg_class, "_full_text");
  PyObject* md5sum = PyObject_GetAttrString(msg_class, "_md5sum");
  if (!md5sum)
  {
    throw std::runtime_error("Class for message " + message_type + " did not contain" +
                             "expected _md5sum attribute.");
  }
  Py_XDECREF(msg_class);

#if PY_VERSION_HEX > 0x03000000
  if (full_text)
  {
    msginfo.full_text.assign(PyUnicode_AsUTF8(full_text));
  }
  msginfo.md5sum.assign(PyUnicode_AsUTF8(md5sum));
#else
  if (full_text)
  {
    msginfo.full_text.assign(PyString_AsString(full_text));
  }
  msginfo.md5sum.assign(PyString_AsString(md5sum));
#endif

  // See https://github.com/ros/ros_comm/issues/344
  // and https://github.com/ros/gencpp/pull/14
  // Valid full_text returned, but it is empty, so insert single line
  if (full_text && msginfo.full_text.empty())
  {
    msginfo.full_text = "\n";
  }

  Py_XDECREF(full_text);
  Py_XDECREF(md5sum);

  return msginfo;
}

}  // namespace rosserial_server
