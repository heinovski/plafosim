# Changelog

## [0.14.1](https://github.com/heinovski/plafosim/releases/tag/v0.14.1) - 2021-11-23
* Added pre-filled status to vehicles
* Added random offset for pre-filled vehicles' execution interval
* Fixed bug in GUI
* Updated CHANGELOG
* Updated README

## [0.14.0](https://github.com/heinovski/plafosim/releases/tag/v0.14.0) - 2021-11-22
* Added packaging with poetry
* Added dry-run flag
* Fixed bug in comparison script
* Updated GUI module
* Updated README
* Updated trace replay script

## [0.13.2](https://github.com/heinovski/plafosim/releases/tag/v0.13.2) - 2021-11-17
* Fixed typo in README
* Updated citation information

## [0.13.1](https://github.com/heinovski/plafosim/releases/tag/v0.13.1) - 2021-11-12
* Added dedicated directory for algorithms
* Added dedicated directory for algorithms
* Fixed ignoring failed CI steps
* Moved CF_Model to mobility module
* Removed obsolete argument
* Updated README

## [0.13.0](https://github.com/heinovski/plafosim/releases/tag/v0.13.0) - 2021-10-20
* Added citation information of accepted poster publication
* Added flag to start GUI in paused mode
* Added integration tests for spawning
* Added new spawn procedure
* Added various checks for input parameters
* Added various tests
* Implemented vectorized lane changes
* Updated correctness tests
* Updated generation of pre-filled platoon
* Updated Platoon Data

## [0.12.0](https://github.com/heinovski/plafosim/releases/tag/v0.12.0) - 2021-09-08
* Added argument for connecting to the GUI later
* Added argument for showing the progress bar
* Added arguments for drawing labels within the GUI
* Added arguments for drawing objects within the GUI
* Added bumpversion
* Added dedicated CI step for validation data upload
* Added dedicated module for writing statistics
* Added dedicated trace files for platoon and member changes
* Added emission class
* Added integration tests
* Added pipeline step for uploading the validation plots to the wiki
* Added snapshot feature
* Cleaned up the code
* Cleaned up trace player
* Extracted GUI code to dedicated module
* Fixed doc image path
* Fixed emission calculation
* Fixed lange change check.
* Fixed leave maneuver
* Fixed references to formulas from literature
* Fixed safe speed calculation
* Renamed CF Model CC to Human
* Updated argument passing
* Updated documentation
* Updated predecessor/successor calculations
* Updated validation comparison
* Updated vehicle spawning
* Vectorized CF Models

## [v0.11.4](https://github.com/heinovski/plafosim/releases/tag/v0.11.4) - 2021-07-21
* Added bootstrapping to calculate confidence interval in comparison
* Added explicit depart speed 0
* Added KS test for desired speed in comparison
* Added profile runs for all cf models to CI
* Fixed lane change duration
* Fixed simulation end upon no vehicles
* Fixed typos
* Renamed depart interval argument
* Updated check of maximum vehicle number
* Updated comparison script
* Updated implicit default value for random seed
* Updated validation scripts

## [v0.11.3](https://github.com/heinovski/plafosim/releases/tag/v0.11.3) - 2021-07-19
* Added check for invalid depart rate
* Added check for useful depart probability
* Fixed calculation of depart position
* Fixed join at the end of the trip
* Fixed trace player
* Removed depart method fixed
* Updated default value of vehicle density parameter
* Updated string representation of platoon

## [v0.11.2](https://github.com/heinovski/plafosim/releases/tag/v0.11.2) - 2021-07-02
* Added maximum trip length
* Added recording of continuous simulation trace
* Added safety check for insertions

## [v0.11.1](https://github.com/heinovski/plafosim/releases/tag/v0.11.1) - 2021-06-24
* Updated result files for platoon related data

## [v0.11.0](https://github.com/heinovski/plafosim/releases/tag/v0.11.0) - 2021-06-21
* Added additional constraints to optimization problem
* Added distinction between desired speed and CC target speed
* Added deprecation warning to communication code
* Added development advice to README
* Added more maneuver abort reasons
* Added solver time limit
* Added statists for solution quality
* Added teleport delay during maneuver
* Fixed argument choices
* Fixed centralized version of speed position algorithm
* Fixed desired headway time
* Fixed metric for successful assignments
* Fixed recording of periodic simulator statistics
* Fixed typos
* Updated CACC model to use direct speed from the leader
* Updated CACC validation to not update platoon's desired speed
* Updated calculation of speed and position deviation
* Updated CI triggers
* Updated CLI arguments
* Updated comments
* Updated condition for front join
* Updated docstring
* Updated formatting
* Updated optimization problem
* Updated parameter variables in simulator
* Updated platoon role in leave maneuver
* Updated properties and variables
* Updated run time calculation
* Updated vehicle tripinfo
* Updated validation scripts
* Updated variable access to gain speed

## [v0.10.0](https://github.com/heinovski/plafosim/releases/tag/v0.10.0) - 2021-06-02
* Added comparison of cf models with single vehicle
* Added comparison to Plexe for CACC
* Added emissions to comparison
* Added pandas for predecessor calculation
* Added predecessor_id to new_speed method
* Added switch for reduced air drag
* Fixed infinite nesting of properties
* Fixed typos
* Fixed wrong execution trigger for formation algorithm
* Updated call to SUMO when using GUI
* Updated CI pipelines and comparisons
* Updated default values for formation thresholds
* Updated depart time of platoon
* Updated docstrings
* Updated logging

## [v0.9.6](https://github.com/heinovski/plafosim/releases/tag/v0.9.6) - 2021-05-25
* Fixed assert in result recording
* Fixed spawning of static platoon

## [v0.9.5](https://github.com/heinovski/plafosim/releases/tag/v0.9.5) - 2021-05-17
* Added argument for sumo GUI config
* Added average candidate metric
* Added checking the maximum speed
* Added maximum approach time
* Added metric for formation iterations
* Fixed issues due to floating precision
* Fixed release names in changelog
* Updated argument help
* Updated CACC calculation

## [v0.9.4](https://github.com/heinovski/plafosim/releases/tag/v0.9.4) - 2021-05-10
* Added flag for updating a platoon's desired speed after a maneuver
* Fixed a lot of typos
* Fixed vehicle moving while join maneuver
* Removed all rounding to fix metrics
* Updated default parameters
* Updated documentation
* Updated maximum teleport distance

## [v0.9.3](https://github.com/heinovski/plafosim/releases/tag/v0.9.3) - 2021-04-28
* Updated copyright headers
* Updated dependencies
* Updated documentation

## [v0.9.2](https://github.com/heinovski/plafosim/releases/tag/v0.9.2) - 2021-04-20
* Added requirements.txt
* Added version argument
* Updated comments
* Updated README

## [v0.9.1](https://github.com/heinovski/plafosim/releases/tag/v0.9.1) - 2021-03-19
* Added drawing ramps and road end
* Added maximum teleport distance
* Fixed candidate metrics

## [v0.9.0](https://github.com/heinovski/plafosim/releases/tag/v0.9.0) - 2021-03-04
* Added all leave cases (simplified)
* Added formation statistics
* Added make space before a teleporting during a join
* Added maneuver statistics
* Added profile run to CI
* Added statistics for optimal solver
* Added statistics for simulator
* Added switch for actions
* Added util module
* Added vectorized collision checks with pandas
* Added vectorized position updates with pandas
* Fixed additional CACC calculation
* Fixed calculations of metrics
* Fixed depart position for pre-filled vehicles
* Fixed generation of random depart speed
* Fixed leaving in the middle of a platoon
* Fixed obsolete depart_time check
* Fixed platoon time metrics
* Fixed randomness when using prefill and GUI
* Fixed record statistics if actions disabled
* Improved code and project structure
* Removed duplicated CACC execution in between steps
* Updated cf model
* Updated comparison script
* Updated error messages
* Updated generation of trips
* Updated insert collision checks
* Updated join maneuver
* Updated log messages
* Updated new speed calculations
* Updated parameters for simulator
* Updated position correction after teleport
* Updated result recording
* Updated warnings

## [v0.8.0](https://github.com/heinovski/plafosim/releases/tag/v0.8.0) - 2020-12-03
* Added communication range between vehicles
* Added desired headway time
* Added emission model
* Added execution intervals for formation algorithms
* Added minimum trip length
* Added more sanity checks for parameter
* Added new ecdfplot in comparison
* Added ortools solver for optimal assignments
* Added switch for disabling result recording for pre-filled vehicles
* Fixed CF Models
* Fixed collisions due to buggy lane change
* Fixed execution paths in scripts
* Fixed position correction after teleport
* Fixed start as platoon
* Small improvements
* Tuned performance of the simulator
* Updated adjustment of the platoon's desired speed to the avg of all members
* Updated CC cf model and comparison
* Updated depart parameters
* Updated exits due to sanity checks
* Updated logging
* Updated random depart/arrival position generation
* Updated simulation with GUI
* Updated the comparison script to sumo
* Updated the join maneuver
* Updated the leave maneuver
* Updated the minimum gap parameter
* Updated units in cli script

## [v0.7.0](https://github.com/heinovski/plafosim/releases/tag/v0.7.0) - 2020-10-15
* Added a more complex join
* Added dedicated modules
* Added default values to all simulator arguments
* Added drawing of infrastructures in the GUI
* Added formation test to CI
* Added missing type hints
* Added (more) configurable result recording
* Added (more) statistics
* Added more tests
* Added simple (speed & position) centralized formation algorithm
* Added type hints
* Added updating platoon followers upon new speed of leader
* Cleaned up imports
* Fixed desired speed for vehicles in a platoon
* Fixed random state when using GUI
* Fixed random state when using pre-fill
* Fixed simulator exit code is ignored in CI
* Fixed teleport in join
* Let formation work with platoons only
* Updated advanced simulation control
* Updated CI definitions
* Updated coloring of vehicles
* Updated copyright headers
* Updated default values for arguments
* Updated join maneuver (adjusting of interfering vehicles)
* Updated logging to use format strings
* Updated methods using only ids to use vehicle instances
* Updated project structure
* Updated testing of Simulator
* Updated trace playing script

## [v0.6.0](https://github.com/heinovski/plafosim/releases/tag/v0.6.0) - 2020-09-22
* Added an abstract base class for formation algorithms
* Added functionality to track a vehicle in the GUI
* Added proper logging
* Added simple join (at back) and leave at front) maneuver
* Added simple (speed & position) distributed formation algorithm
* Added support for infrastructure
* Moved formation logic to composition
* Split up vehicle.py
* Updated platoon class
* Updated traffic generation

## [v0.5.0](https://github.com/heinovski/plafosim/releases/tag/v0.5.0) - 2020-09-04
* Added ACC car-following model
* Added CACC car-following model
* Added comparison to Sumo for ACC
* Added random seed
* Added step log
* Improved collision check
* Reworked lane change safety check
* Updated comparison script

## [v0.4.0](https://github.com/heinovski/plafosim/releases/tag/v0.4.0) - 2020-08-10
* Added comparison step to CI
* Added execution step to CI
* Added script to compare the simulator to Sumo
* Added simple lane change model
* Updated car-following model

## [v0.3.0](https://github.com/heinovski/plafosim/releases/tag/v0.3.0) - 2020-08-03
* Added blocked warning
* Added platoon class
* Added testing framework
* Improved the result recording
* Updated the GUI
* Updated trip generation
* Update the step log

## [v0.2.0](https://github.com/heinovski/plafosim/releases/tag/v0.2.0) - 2020-07-06
* Added functionality to removed finished vehicles
* Added neighbor table stub
* Added simple live GUI by using Sumo
* Added simple vehicle trace player
* Reworked internal data structure for vehicles
* Updated car-following model
* Updated messaging
* Updated result recording

## [v0.1.0](https://github.com/heinovski/plafosim/releases/tag/v0.1.0) - 2020-06-04
* Began a new project
* Added a VehicleType class
* Added CLI script with arguments
* Added Krauss' car-following model
* Added parameters for vehicles, trips, and the simulator
* Added PlatooningVehicle class
* Added PlatoonRole enum
* Added simple communication functionality between vehicles
* Added simple platoon advertising functionality
* Added simple result recording
* Added Simulator class
* Added Vehicle class
