// Adapted from libuipc stable_neo_hookean (Apache-2.0); local LogNeoHookean extension.
#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::constitution
{
class PyLogNeoHookean
{
  public:
    PyLogNeoHookean(py::module& m);
};
}  // namespace pyuipc::constitution
