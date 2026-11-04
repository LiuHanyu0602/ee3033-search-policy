# Formal target interface contract — first integration checkpoint

This package defines wire formats only. It does NOT implement nodes, services,
mission transitions or hardware acceptance. /target/set_class does not exist
until an adapter server is implemented and running.

The existing search package uses ament_python. ROSIDL interfaces must be built
in a sibling ament_cmake package, not by adding .srv files to setup.py.

For Git tracking, store this package under
`interface_packages/ee3033_search_policy_interfaces/` in the repository.
For colcon discovery, create a sibling symlink in workspace src to that directory.
Do not replace an existing interface package or symlink without inspecting it.

Observation header uses original image stamp and optical frame. Estimate header
uses measurement stamp and map frame. Every configuration gets a unique episode_id;
receivers must reject old episodes and old timestamps. Confirmation and coordinate
validity are different fields; detection alone never authorizes return home.

Next required code: adapter service+publisher, stamped localization consumer,
mission stop/report coordinator, validated stop terminal handoff from search manager,
launch wiring and regression/integration tests. Existing tested MLP stays unchanged.

Build evidence: author checked XML and schema text only. Humble build/import must
be performed locally. No live service, navigation or robot test has been run.
