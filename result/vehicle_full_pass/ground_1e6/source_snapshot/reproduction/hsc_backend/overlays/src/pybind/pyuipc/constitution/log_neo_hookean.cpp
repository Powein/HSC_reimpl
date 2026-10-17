// Adapted from libuipc stable_neo_hookean (Apache-2.0); local LogNeoHookean extension.
#include <pyuipc/constitution/log_neo_hookean.h>
#include <uipc/constitution/log_neo_hookean.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
using namespace uipc::geometry;
PyLogNeoHookean::PyLogNeoHookean(py::module& m)
{
    auto class_LogNeoHookean = py::class_<LogNeoHookean, FiniteElementConstitution>(
        m, "LogNeoHookean", R"(LogNeoHookean constitution for non-inverting logarithmic neo-Hookean hyperelastic material.)");

    class_LogNeoHookean.def(py::init<const Json&>(),
                               py::arg("config") = LogNeoHookean::default_config(),
                               R"(Create a LogNeoHookean constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_LogNeoHookean.def_static("default_config",
                                      &LogNeoHookean::default_config,
                                      R"(Get the default LogNeoHookean configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_LogNeoHookean.def("apply_to",
                               [](LogNeoHookean& self,
                                  SimplicialComplex& sc,
                                  const ElasticModuli& moduli,
                                  Float mass_density)
                               { self.apply_to(sc, moduli, mass_density); },
                               py::arg("sc"),
                               py::arg("moduli") =
                                   ElasticModuli::youngs_poisson(120.0_kPa, 0.49),
                               py::arg("mass_density") = 1.0e3,
                               R"(Apply LogNeoHookean constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    moduli: ElasticModuli (default: Young's modulus 120.0 kPa, Poisson's ratio 0.49).
    mass_density: Mass density (default: 1000.0).)");
}
}  // namespace pyuipc::constitution
