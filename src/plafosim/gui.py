#
# Copyright (c) 2020-2022 Julian Heinovski <heinovski@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

import logging
import os

LOG = logging.getLogger(__name__)
TRACI_SUPPORTED_VERSIONS = {
    "1.6.0": 20,
}


def start_gui(config: str, play: bool = True):
    """
    Starts the GUI.

    Parameters
    ----------
    config : str
        The name of the configuration file
    """

    binary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    command = [
        binary,
        '-Q',
        '-c', config,
        '--collision.mingap-factor', '0',
        '--collision.action', 'none',
        '--start', str(play),
    ]
    import traci
    traci.start(command)

    # TODO perform this check before the actual GUI has been opened
    traci_api = sumo = None
    try:
        traci_api, sumo = traci.getVersion()
    except AttributeError:
        pass
    try:
        traci_api, sumo = traci.main.getVersion()
    except AttributeError:
        pass
    sumo = sumo.split(' ')[1]
    assert traci_api
    assert sumo
    assert traci_api in TRACI_SUPPORTED_VERSIONS.values(), f"You are using an unsupported TraCI version ({traci_api})!"
    # NOTE: we allow other SUMO vesions with the correct TraCI API version (e.g., development versions).
    # However, these versions are untested and thus might lead lead to unexpected behaior.
    if sumo not in TRACI_SUPPORTED_VERSIONS.keys():
        LOG.warning(f"You are using an untested SUMO version ({sumo})!")


def set_gui_window(road_length: int):
    """
    Set the window of the GUI according to the road length.

    Parameters
    ----------
    road_length : int
        The length of the road in m
    """
    import traci
    traci.gui.setBoundary(
        traci.gui.DEFAULT_VIEW,
        xmin=0,
        ymin=-25000,
        xmax=road_length,
        ymax=25000,
    )
    # TODO set zoom?
    # traci.gui.setZoom(traci.gui.DEFAULT_VIEW, 1000)


def add_gui_vehicle(vid: int, position: float, lane: int, speed: float, color: tuple = (0, 255, 0), track: bool = False):
    """
    Adds a vehicle to the GUI.

    Parameters
    ----------
    vid : int
        The vehicle's id
    position : float
        The vehicle's current position
    lane : int
        The vehicle's current lane
    speed : float
        The vehicle's current speed
    color : tuple
        The vehicle's current color
    track : bool
        Whether to track this vehicle within the GUI
    """

    LOG.trace(f"Adding vehicle {vid} at {position},{lane} with {speed},{color}")
    import traci
    if vid not in traci.vehicle.getIDList():
        traci.vehicle.add(
            vehID=str(vid),
            routeID='route',
            typeID='vehicle',
            departLane=lane,
            departPos=position,
            departSpeed=speed,
        )
        traci.vehicle.setColor(str(vid), color)
        traci.vehicle.setSpeedMode(str(vid), 0)
        traci.vehicle.setLaneChangeMode(str(vid), 0)
        # track vehicle
        if track:
            traci.gui.trackVehicle(traci.gui.DEFAULT_VIEW, str(vid))
            traci.gui.setZoom(traci.gui.DEFAULT_VIEW, 700000)


def move_gui_vehicle(vid: int, position: float, lane: int, speed: float):
    """
    Moves a vehicle in the GUI.

    Parameters
    ----------
    vid : int
        The id of the vehicle to change
    position : float
        The vehicle's new position
    lane : int
        The vehicle's new lane
    speed : float
        The vehicle's new speed
    """

    import traci
    LOG.trace(f"Moving vehicle {vid} to {position},{lane} with {speed}")
    traci.vehicle.setSpeed(vehID=str(vid), speed=speed)
    traci.vehicle.moveTo(vehID=str(vid), laneID=f'edge_0_0_{lane}', pos=position)


def gui_step(target_step: int):
    """
    Increases the simulation step in the GUI.

    Parameters
    ----------
    target_step : int
        The target simulation step
    """

    import traci
    traci.simulationStep(target_step)
    assert traci.simulation.getTime() == float(target_step)


def change_gui_vehicle_color(vid: int, color: tuple):
    """
    Changes the color of a vehicle in the GUI.

    Parameters
    ----------
    vid : int
        The id of the vehicle to change
    color : tuple
        The color (R, G, B) to use for the vehicle
    """

    import traci
    LOG.trace(f"Changing color of vehicle {vid} to {color}")
    traci.vehicle.setColor(str(vid), color)


def remove_gui_vehicle(vid: int):
    """
    Removes a vehicle from the GUI.

    Parameters
    ----------
    vid : int
        The id of the vehicle to remove
    """

    import traci
    LOG.trace(f"Removing vehicle {vid}")
    traci.vehicle.remove(str(vid), 2)


def prune_vehicles(keep_vids: list):
    """
    Prunes vehicles from the GUI.

    Parameters
    ----------
    keep_vids : list
        The ids of the vehicle that should be kept
    """

    import traci
    for vid in set(map(int, traci.vehicle.getIDList())) - set(keep_vids):
        remove_gui_vehicle(vid)


def close_gui():
    """
    Closes the GUI.

    """

    import traci
    traci.close(False)


def draw_ramps(road_length: int, interval: int, labels: bool):
    """
    Draws on-/off-ramps in the GUI.

    Parameters
    ----------
    road_length : int
        The length of the road in m
    interval : int
        The ramp interval in m
    labels : bool
        Whether to draw ramp labels
    """

    y = 241
    color = (0, 0, 0)
    width = 4
    height = 150

    import traci
    for x in range(0, road_length + 1, interval):
        traci.polygon.add(f"ramp-{x}", [
            (x - width / 2, y),  # top left
            (x + width / 2, y),  # top right
            (x + width / 2, y - height),  # bottom right
            (x - width / 2, y - height)   # bottom left
        ], color, fill=True)
        if labels:
            traci.poi.add(
                f"Ramp at {x}m",
                x=x,
                y=y - height - 10,
                color=(51, 128, 51),
            )


def draw_road_end(road_length: int, label: bool):
    """
    Draws the end of the road in the GUI.

    Parameters
    ----------
    road_length : int
        The length of the road in m
    label : bool
        Whether to draw a label
    """

    y_top = 340
    y_bottom = 241
    width = 4
    color = (255, 0, 0)

    import traci
    traci.polygon.add("road-end", [
        (road_length - width / 2, y_bottom),  # bottom left
        (road_length + width / 2, y_bottom),  # bottom right
        (road_length + width / 2, y_top),  # top right
        (road_length - width / 2, y_top)  # top left
    ], color, fill=True, layer=3)
    if label:
        traci.poi.add(
            "Road End",
            x=road_length + 50,
            y=300,
            color=(51, 128, 51),
        )


def draw_infrastructures(infrastructures: list, labels: bool):
    """
    Draws infrastructures in the GUI.

    Parameters
    ----------
    infrastructures : list
        The list of infrastructure objects to add
    labels : bool
        Whether to draw infrastructure labels
    """

    y = 280
    width = 20
    color = (0, 0, 255)

    import traci
    for infrastructure in infrastructures:
        from .infrastructure import Infrastructure
        assert isinstance(infrastructure, Infrastructure)
        iid = str(infrastructure.iid)
        position = infrastructure.position

        # add infrastructure
        if iid not in traci.polygon.getIDList():
            traci.polygon.add(f"rsu-{iid}", [
                (position - width / 2, y),  # bottom left
                (position + width / 2, y),  # bottom right
                (position + width / 2, y + width),  # top right
                (position - width / 2, y + width)  # top left
            ], color, fill=True)
            if labels:
                traci.poi.add(
                    f"RSU {iid}",
                    x=position,
                    y=y + width + 10,
                    color=(51, 128, 51),
                )
