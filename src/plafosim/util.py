#
# Copyright (c) 2020-2024 Julian Heinovski <heinovski@ccs-labs.org>
#
# SPDX-License-Identifier: GPL-3.0-or-later
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
import math
import os
import textwrap


def find_resource(path: str) -> str:
    """
    Find the resouces under relpath locally or as a packaged resource.

    Parameters
    ----------
    path : str
        The path to search for the ressource

    Returns
    -------
    str : The path of the resource
    """
    # prioritize local paths
    if os.path.exists(path):
        return path

    resource_path = os.path.join(os.path.dirname(__file__), path)
    if not os.path.exists(resource_path):
        raise ValueError(
            f"Path is not a local file or packaged resource: {path}."
        )
    return str(resource_path)


def round_to_next_base(value: float, base: float) -> float:
    """
    Round a value to the next base value.

    Parameters
    ----------
    value : float
        The value to round
    base : float
        The base value to round to

    Returns
    -------
    float : The rounded value
    """
    return base * math.ceil(value / base)


def rgb2hex(c_rgb: tuple) -> str:
    """
    Convert a color in RGB values to a color in hex values.

    Parameters
    ----------
    c_rgb : tuple(int, int, int)
        The color in RGB values

    Returns
    -------
    str : The color in hex values
    """

    assert len(c_rgb) == 3
    for c in c_rgb:  # pylint: disable=C0103
        assert isinstance(c, int) and 0 <= c <= 255

    return '#' + ''.join(f'{i:02X}' for i in c_rgb)


def hex2rgb(c_hex: str) -> tuple:
    """
    Convert a color in hex values to a color in RGB values.

    Parameters
    ----------
    c_hex : str
        The color in hex values

    Returns
    -------
    tuple(int, int, int) : The color in RGB values
    """

    assert c_hex[0] == '#'
    assert len(c_hex[1:]) == 6

    return tuple(int(i, 16) for i in textwrap.wrap(c_hex[1:], 2))


def assert_index_equal(one, two) -> bool:
    """
    Ensure the indices of two Sequences/DataFrames are equal.

    Parameters
    ----------
    one : pandas.Sequence / pandas.DataFrame
        The first object for the comparison
    two : pandas.Sequence / pandas.DataFrame
        The second object for the comparsion

    Returns
    -------
    bool : Whether the two indices are equal
    """

    # TODO use pandas.testing.assert_index_equal?
    assert list(one.index) == list(two.index)


def speed2distance(speed: float, time_interval: float = 1.0) -> float:
    """
    Convert a driving speed to a distance driven within a given time interval.

    Parameters
    ----------
    speed : float
        The speed to be converted
    time_interval : float, optional
        The time to consider

    Returns
    -------
    float : The converted value
    """

    return speed * time_interval


def distance2speed(distance: float, time_interval: float = 1.0) -> float:
    """
    Convert a driven distance to a driving speed for a given time interval.

    Parameters
    ----------
    distance : float
        The distance to be converted
    time_interval : float, optional
        The time to consider

    Returns
    -------
    float : The converted value
    """

    return distance / time_interval


def acceleration2speed(acceleration: float, time_interval: float = 1.0) -> float:
    """
    Convert an acceleration to a driving speed for a given time interval.

    Parameters
    ----------
    acceleration : float
        The acceleration to be converted
    time_interval : float, optional
        The time to consider

    Returns
    -------
    float : The converted value
    """

    return acceleration * time_interval


def speed2acceleration(speed_from: float, speed_to: float, time_interval: float = 1.0) -> float:
    """
    Convert a speed range to an acceleration within a given time interval.

    Parameters
    ----------
    speed_from : float
        The initial speed
    speed_to : float
        The target speed
    time_interval : float, optional
        The time to consider

    Returns
    -------
    float : The converted value
    """

    return (speed_to - speed_from) / time_interval


def add_logging_level(level_name: str, level_num: int, method_name: str = None):
    """
    Comprehensively adds a new logging level to the `logging` module and the currently configured logging class.

    `level_name` becomes an attribute of the `logging` module with the value `level_num`.
    `method_name` becomes a convenience method for both `logging` itself and the class returned by `logging.getLoggerClass()` (usually just `logging.Logger`).
    If `method_name` is not specified, `level_name.lower()` is used.

    To avoid accidental clobberings of existing attributes, this method will raise an `AttributeError` if the level name is already an attribute of the `logging` module or if the method name is already present.

    Taken from https://stackoverflow.com/a/35804945.

    Parameters
    ----------
    level_name : str
        The name of the level to add
    level_num : int
        The number of the level to add
    method_name : str
        The name of the method for the level to add
    """

    if not method_name:
        method_name = level_name.lower()

    if hasattr(logging, level_name):
        raise AttributeError('{} already defined in logging module'.format(level_name))
    if hasattr(logging, method_name):
        raise AttributeError('{} already defined in logging module'.format(method_name))
    if hasattr(logging.getLoggerClass(), method_name):
        raise AttributeError('{} already defined in logger class'.format(method_name))

    # This method was inspired by the answers to Stack Overflow post
    # http://stackoverflow.com/q/2183233/2988730, especially
    # http://stackoverflow.com/a/13638084/2988730
    def log_for_level(self, message, *args, **kwargs):
        if self.isEnabledFor(level_num):
            self._log(level_num, message, args, **kwargs)

    def log_to_root(message, *args, **kwargs):
        logging.log(level_num, message, *args, **kwargs)

    logging.addLevelName(level_num, level_name)
    setattr(logging, level_name, level_num)
    setattr(logging.getLoggerClass(), method_name, log_for_level)
    setattr(logging, method_name, log_to_root)


class FakeLog:
    """
    A fake logger, hide LOG behind this and be happy.
    """

    def trace(self, *arg, **kwd):  # noqa D102
        pass

    def debug(self, *arg, **kwd):  # noqa D102
        pass

    def info(self, *arg, **kwd):  # noqa D102
        pass

    def warning(self, *arg, **kwd):  # noqa D102
        pass

    def warn(self, *arg, **kwd):  # noqa D102
        pass

    def error(self, *arg, **kwd):  # noqa D102
        pass

    def fatal(self, *arg, **kwd):  # noqa D102
        pass


FAKELOG = FakeLog()  # default fake logger
