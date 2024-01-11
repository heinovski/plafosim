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
import subprocess
import sys


def main():
    """
    The main entry point of PlaFoSim's img2video script.

    This script uses ffmpeg.
    """

    if len(sys.argv) != 3:
        print(f"A script for creating a video from screenshots with fmpeg.\n\nusage: {sys.argv[0]} 'prefix_*.png' video.mp4")
        sys.exit()

    print(f"Creating video {sys.argv[2]} from images with glob pattern {sys.argv[1]} ...")

    # use ffmpeg to create a video from screenshot
    cmd = [
        'ffmpeg',
        '-framerate',
        '25',
        '-pattern_type',
        'glob',
        '-i',
        sys.argv[1],
        '-c:v',
        'libx264',
        '-profile:v',
        'high',
        '-crf',
        '20',
        '-pix_fmt',
        'yuv420p',
        sys.argv[2],
    ]

    result = subprocess.run(cmd, stdout=subprocess.PIPE)
    print(result.stdout.decode('utf-8'))


if __name__ == "__main__":
    sys.exit(main())
