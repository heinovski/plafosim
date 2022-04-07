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
import math
import os
import textwrap


def find_resource(path: str) -> str:
    """
    Find the resouces under relpath locally or as a packaged resource.
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


def round_to_next_base(x: float, base: float) -> float:
    return base * math.ceil(x / base)


def rgb2hex(rgb: tuple) -> str:
    assert len(rgb) == 3
    for c in rgb:
        assert (type(c) == int and 0 <= c <= 255)

    return '#' + ''.join(f'{i:02X}' for i in rgb)


def hex2rgb(hex: str) -> tuple:
    assert hex[0] == '#'
    assert len(hex[1:]) == 6

    return tuple(int(i, 16) for i in textwrap.wrap(hex[1:], 2))


def assert_index_equal(a, b):
    """
    Ensures the indeces of two Sequences/DataFrames are equal.

    Parameters
    ----------
    a : pandas.Sequence / pandas.DataFrame
    b : pandas.Sequence / pandas.DataFrame
    """

    assert list(a.index) == list(b.index)


def speed2distance(speed: float, time_interval: float = 1.0) -> float:
    """
    Converts a driving speed to a distance driven within a given time interval.

    Parameters
    ----------
    speed : float
        The speed to be converted
    time_interval : float, optional
        The time to consider
    """

    return speed * time_interval


def distance2speed(distance: float, time_interval: float = 1.0) -> float:
    """
    Converts a driven distance to a driving speed for a given time interval.

    Parameters
    ----------
    distance : float
        The distance to be converted
    time_interval : float, optional
        The time to consider
    """

    return distance / time_interval


def acceleration2speed(acceleration: float, time_interval: float = 1.0) -> float:
    """
    Converts an acceleration to a driving speed for a given time interval.

    Parameters
    ----------
    acceleration : float
        The acceleration to be converted
    time_interval : float, optional
        The time to consider
    """

    return acceleration * time_interval


def speed2acceleration(speed_from: float, speed_to: float, time_interval: float = 1.0) -> float:
    """
    Converts a speed range to an acceleration within a given time interval.

    Parameters
    ----------
    speed_from : float
        The initial speed
    speed_to : float
        The target speed
    time_interval : float, optional
        The time to consider
    """

    return (speed_to - speed_from) / time_interval


def addLoggingLevel(levelName, levelNum, methodName=None):
    """
    https://stackoverflow.com/a/35804945

    Comprehensively adds a new logging level to the `logging` module and the currently configured logging class.

    `levelName` becomes an attribute of the `logging` module with the value `levelNum`. `methodName` becomes a convenience method for both `logging` itself and the class returned by `logging.getLoggerClass()` (usually just `logging.Logger`). If `methodName` is not specified, `levelName.lower()` is used.

    To avoid accidental clobberings of existing attributes, this method will raise an `AttributeError` if the level name is already an attribute of the `logging` module or if the method name is already present
    """

    if not methodName:
        methodName = levelName.lower()

    if hasattr(logging, levelName):
        raise AttributeError('{} already defined in logging module'.format(levelName))
    if hasattr(logging, methodName):
        raise AttributeError('{} already defined in logging module'.format(methodName))
    if hasattr(logging.getLoggerClass(), methodName):
        raise AttributeError('{} already defined in logger class'.format(methodName))

    # This method was inspired by the answers to Stack Overflow post
    # http://stackoverflow.com/q/2183233/2988730, especially
    # http://stackoverflow.com/a/13638084/2988730
    def logForLevel(self, message, *args, **kwargs):
        if self.isEnabledFor(levelNum):
            self._log(levelNum, message, args, **kwargs)

    def logToRoot(message, *args, **kwargs):
        logging.log(levelNum, message, *args, **kwargs)

    logging.addLevelName(levelNum, levelName)
    setattr(logging, levelName, levelNum)
    setattr(logging.getLoggerClass(), methodName, logForLevel)
    setattr(logging, methodName, logToRoot)


class FakeLog:
    """A fake logger, hide LOG behind this and be happy."""
    def trace(self, *arg, **kwd):
        pass

    def debug(self, *arg, **kwd):
        pass

    def info(self, *arg, **kwd):
        pass

    def warning(self, *arg, **kwd):
        pass

    def warn(self, *arg, **kwd):
        pass

    def error(self, *arg, **kwd):
        pass

    def fatal(self, *arg, **kwd):
        pass


FAKELOG = FakeLog()  # default fake logger
