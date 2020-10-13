# Changelog

## v0.7.0, 2020-10-15
- Added simple centralized formation algorithm (speed & position)
- Added updating platoon followers upon new speed of leader
- Added default values to all simulator arguments
- Added more tests
- Added (more) configurable result recording
- Added type hints
- Added drawing of infrastructures in the GUI
- Updated join maneuver (adjusting of interfering vehicles)
- Updated logging to use format strings
- Updated CI definitions
- Updated coloring of vehicles
- Updated default values for arguments
- Updated copyright headers
- Updated project structure
- Updated methods using only ids to use vehicle instances
- Updated trace playing script
- Fixed random state when using GUI
- Fixed random state when using pre-fill

## v0.6.0, 2020-09-22
- Updated traffic generation
- Added simple join (at back) maneuver
- Added an abstract base class for formation algorithms
- Added simple distributed formation algorithm (speed & position)
- Added simple leave (at front) maneuver
- Added support for infrastructure
- Updated platoon class
- Added proper logging
- Added functionality to track a vehicle in the gui

## v0.5.0, 2020-09-04
- Added ACC car-following model
- Added CACC car-following model
- Reworked lane change safety check
- Improved collision check
- Added random seed to simulator (and CI execution)
- Added progress bars
- Reworked comparison to Sumo
- Added comparison to Sumo for ACC

## v0.4.0, 2020-08-10
- Added simple lane change model
- Added script to compare the simulator to Sumo
- Updated car-following model
- Added execution step to CI
- Added comparison step to CI

## v0.3.0, 2020-08-03
- Updated trip generation
- Added platoon class
- Added blocked warning
- Added testing framework
- Improved the result recording
- Update the step log
- Updated the GUI

## v0.2.0, 2020-07-06
- Updated car-following model
- Updated messaging
- Updated result recording
- Reworked internal data structure for vehicles
- Added functionality to removed finished vehicles
- Added simple vehicle trace player
- Added simple live GUI by using Sumo
- Added neighbor table stub

## v0.1.0, 2020-06-04 -- Initial release.
- Began a new project
- Added a VehicleType class
- Added Vehicle class
- Added Simulator class
- Added Krauss' car-following model
- Added simple communication functionality between vehicles
- Added PlatoonRole enum
- Added PlatooningVehicle class
- Added CLI script with arguments
- Added parameters for vehicles, trips, and the simulator
- Added simple result recording
- Added simple platoon advertising functionality
