# Changelog

## [0.13.2](https://github.com/heinovski/plafosim/releases/tag/v0.13.2) - 2021-11-17
* Fixed typo in README
* Updated citation information

## [0.13.1](https://github.com/heinovski/plafosim/releases/tag/v0.13.1) - 2021-11-12
* Added dedicated directory for algorithms
* Added dedicated directory for algorithms (#376)
* Fixed ignoring failed CI steps (#380)
* Moved CF_Model to mobility module (#377)
* Removed obsolete argument
* Updated README

## [0.13.0](https://github.com/heinovski/plafosim/releases/tag/v0.13.0) - 2021-10-20
* Added citation information of accepted poster publication
* Added flag to start gui in paused mode
* Added integration tests for spawning (#348)
* Added new spawn procedure (#374)
* Added various checks for input parameters
* Added various tests
* Implemented vectorized lane changes (#372)
* Updated correctness tests
* Updated generation of pre-filled platoon
* Updated Platoon Data (#373)

## [0.12.0](https://github.com/heinovski/plafosim/releases/tag/v0.12.0) - 2021-09-08
* Added argument for connecting to the gui later (#340)
* Added argument for showing the progress bar
* Added arguments for drawing labels within the GUI
* Added arguments for drawing objects within the GUI
* Added bumpversion
* Added dedicated CI step for validation data upload
* Added dedicated module for writing statistics (#355)
* Added dedicated trace files for platoon and member changes
* Added emission class (#337)
* Added integration tests (#347, #367)
* Added pipeline step for uploading the validation plots to the wiki (#335)
* Added snapshot feature (#341)
* Cleaned up the code
* Cleaned up trace player
* Extracted GUI code to dedicated module (#350)
* Fixed doc image path (#318)
* Fixed emission calculation
* Fixed lange change check. (#349)
* Fixed leave maneuver (#322)
* Fixed references to formulas from literature
* Fixed safe speed calculation (#324)
* Renamed CF Model CC to Human (#369)
* Updated argument passing (#365)
* Updated documentation
* Updated predecessor/successor calculations
* Updated validation comparison (#325, #326, #333, #336, #362)
* Updated vehicle spawning (#321, #353)
* Vectorized CF Models (#323, #354)

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
* Added maximum trip length (#315)
* Added recording of continuous simulation trace (#316)
* Added safety check for insertions (#265, #317)

## [v0.11.1](https://github.com/heinovski/plafosim/releases/tag/v0.11.1) - 2021-06-24
* Updated result files for platoon related data

## [v0.11.0](https://github.com/heinovski/plafosim/releases/tag/v0.11.0) - 2021-06-21
* Added additional constraints to optimization problem (#304)
* Added distinction between desired speed and CC target speed (#309)
* Added deprecation warning to communication code
* Added development advice to README
* Added more maneuver abort reasons
* Added solver time limit
* Added statists for solution quality
* Added teleport delay during maneuver (#301)
* Fixed argument choices
* Fixed centralized version of speed position algorithm
* Fixed desired headway time
* Fixed metric for successful assignments
* Fixed recording of periodic simulator statistics
* Fixed typos
* Updated CACC model to use direct speed from the leader (#302)
* Updated CACC validation to not update platoon's desired speed (#306)
* Updated calculation of speed and position deviation (#310)
* Updated CI triggers
* Updated CLI arguments
* Updated comments
* Updated condition for front join
* Updated docstring
* Updated formatting
* Updated optimization problem
* Updated parameter variables in simulator
* Updated platoon role in leave maneuver (#307)
* Updated properties and variables (#298)
* Updated run time calculation
* Updated vehicle tripinfo
* Updated validation scripts
* Updated variable access to gain speed

## [v0.10.0](https://github.com/heinovski/plafosim/releases/tag/v0.10.0) - 2021-06-02
* Added comparison of cf models with single vehicle (#229, #293)
* Added comparison to Plexe for CACC (#84)
* Added emissions to comparison (#177)
* Added pandas for predecessor calculation (#283)
* Added predecessor_id to new_speed method
* Added switch for reduced air drag
* Fixed infinite nesting of properties
* Fixed typos
* Fixed wrong execution trigger for formation algorithm
* Updated call to SUMO when using GUI
* Updated CI pipelines and comparisons (#279, #280, #287)
* Updated default values for formation thresholds
* Updated depart time of platoon
* Updated docstrings
* Updated logging

## [v0.9.6](https://github.com/heinovski/plafosim/releases/tag/v0.9.6) - 2021-05-25
* Fixed assert in result recording (#278)
* Fixed spawning of static platoon (#277)

## [v0.9.5](https://github.com/heinovski/plafosim/releases/tag/v0.9.5) - 2021-05-17
* Added argument for sumo gui config
* Added average candidate metric (#276)
* Added checking the maximum speed
* Added maximum approach time
* Added metric for formation iterations
* Fixed issues due to floating precision (#275)
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
* Added drawing ramps and road end (#253)
* Added maximum teleport distance (#267)
* Fixed candidate metrics (#266)

## [v0.9.0](https://github.com/heinovski/plafosim/releases/tag/v0.9.0) - 2021-03-04
* Added all leave cases (simplified) (#206)
* Added formation statistics
* Added make space before a teleporting during a join (#232)
* Added maneuver statistics (#226)
* Added profile run to CI
* Added statistics for optimal solver (#230)
* Added statistics for simulator
* Added switch for actions
* Added util module
* Added vectorized collision checks with pandas (#208)
* Added vectorized position updates with pandas (#211)
* Fixed additional CACC calculation (#212)
* Fixed calculations of metrics
* Fixed depart position for pre-filled vehicles
* Fixed generation of random depart speed (#254)
* Fixed leaving in the middle of a platoon (#241)
* Fixed obsolete depart_time check (#209)
* Fixed platoon time metrics (#234)
* Fixed randomness when using prefill and GUI
* Fixed record statistics if actions disabled (#237)
* Improved code and project structure
* Removed duplicated CACC execution in between steps (#238)
* Updated cf model
* Updated comparison script (#228)
* Updated error messages
* Updated generation of trips (#252, #256)
* Updated insert collision checks
* Updated join maneuver
* Updated log messages
* Updated new speed calculations
* Updated parameters for simulator
* Updated position correction after teleport
* Updated result recording (#222, #223)
* Updated warnings

## [v0.8.0](https://github.com/heinovski/plafosim/releases/tag/v0.8.0) - 2020-12-03
* Added communication range between vehicles (#167)
* Added desired headway time
* Added emission model (#50)
* Added execution intervals for formation algorithms (#166)
* Added minimum trip length (#170, #187)
* Added more sanity checks for parameter
* Added new ecdfplot in comparison (#168)
* Added ortools solver for optimal assignments (#127)
* Added switch for disabling result recording for pre-filled vehicles (#190)
* Fixed CF Models (#185)
* Fixed collisions due to buggy lane change (#183)
* Fixed execution paths in scripts (#184)
* Fixed position correction after teleport (#188)
* Fixed start as platoon (#182)
* Small improvements (#174)
* Tuned performance of the simulator (#172)
* Updated adjustment of the platoon's desired speed to the avg of all members (#162)
* Updated CC cf model and comparison (#176)
* Updated depart parameters (#175)
* Updated exits due to sanity checks
* Updated logging (#137, #161)
* Updated random depart/arrival position generation
* Updated simulation with gui
* Updated the comparison script to sumo
* Updated the join maneuver
* Updated the leave maneuver
* Updated the minimum gap parameter
* Updated units in cli script

## [v0.7.0](https://github.com/heinovski/plafosim/releases/tag/v0.7.0) - 2020-10-15
* Added a more complex join (#124)
* Added dedicated modules (#111)
* Added default values to all simulator arguments
* Added drawing of infrastructures in the GUI (#130)
* Added formation test to CI (#101)
* Added missing type hints (#120)
* Added (more) configurable result recording
* Added (more) statistics (#143, #145)
* Added more tests
* Added simple (speed & position) centralized formation algorithm (#109)
* Added type hints
* Added updating platoon followers upon new speed of leader (#95)
* Cleaned up imports (#131)
* Fixed desired speed for vehicles in a platoon (#139)
* Fixed random state when using GUI (#136)
* Fixed random state when using pre-fill
* Fixed simulator exit code is ignored in CI (#146)
* Fixed teleport in join (#135)
* Let formation work with platoons only (#123)
* Updated advanced simulation control (#133)
* Updated CI definitions
* Updated coloring of vehicles
* Updated copyright headers
* Updated default values for arguments
* Updated join maneuver (adjusting of interfering vehicles)
* Updated logging to use format strings
* Updated methods using only ids to use vehicle instances
* Updated project structure
* Updated testing of Simulator (#126)
* Updated trace playing script

## [v0.6.0](https://github.com/heinovski/plafosim/releases/tag/v0.6.0) - 2020-09-22
* Added an abstract base class for formation algorithms
* Added functionality to track a vehicle in the gui (#89)
* Added proper logging (#92)
* Added simple join (at back) and leave at front) maneuver (#85)
* Added simple (speed & position) distributed formation algorithm (#53)
* Added support for infrastructure (#96)
* Moved formation logic to composition (#103)
* Split up vehicle.py (#87)
* Updated platoon class (#93)
* Updated traffic generation (#79)

## [v0.5.0](https://github.com/heinovski/plafosim/releases/tag/v0.5.0) - 2020-09-04
* Added ACC car-following model (#51)
* Added CACC car-following model (#61)
* Added comparison to Sumo for ACC
* Added random seed (#78, #81)
* Added step log (#75)
* Improved collision check
* Reworked lane change safety check
* Updated comparison script (#71, #73, #74)

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
