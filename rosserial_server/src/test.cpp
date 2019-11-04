
#include "Python.h"
#include <iostream>

int main(int argc, char* argv[])
{
  Py_Initialize();
  PyObject* module = PyImport_ImportModule("std_msgs.msg");
  assert(module);
  PyObject* msg = PyObject_GetAttrString(module, "Float32");
  assert(msg);

  PyObject* md5sum = PyObject_GetAttrString(msg, "_md5sum");
  PyObject* fulltext = PyObject_GetAttrString(msg, "_full_text");
#if PY_VERSION_HEX > 0x03000000
  std::cout << "MD5: " << PyUnicode_AsUTF8(md5sum) << "\n";
  std::cout << "Text: " << PyUnicode_AsUTF8(fulltext) << "\n";
#else
  std::cout << "MD5: " << PyString_AsString(md5sum) << "\n";
  std::cout << "Text: " << PyString_AsString(fulltext) << "\n";
#endif
  Py_XDECREF(msg);
  Py_XDECREF(md5sum);
  Py_XDECREF(fulltext);
  Py_Finalize();

  return 0;
}
