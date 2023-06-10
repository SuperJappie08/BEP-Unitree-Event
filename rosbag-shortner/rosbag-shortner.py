#!/usr/bin/python3

import sys

if "rosbag" not in sys.modules.keys():
    sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

del sys

from rosbag import Bag
import rosbag.bag as bag
import argparse
import tqdm
from pathlib import Path
from typing import List, Tuple, Optional, Union


def main():
    parser = argparse.ArgumentParser(
        "rosbag-shortner.py", description="Cut rosbags to size."
    )
    parser.add_argument("infile", metavar="INFILE", type=Path, help="Target file")
    parser.add_argument(
        "endtime",
        metavar="ENDTIMESTAMP",
        type=int,
        help="The final timestep in nanoseconds. (Records until ENDTIMESTAMP + 35E+6 for an extra frame)",
    )
    dest_group = parser.add_argument_group(title="Destination")
    dest_group.add_argument(
        "--dest",
        dest="outfile",
        metavar="OUTFILE",
        type=Path,
        help="Destination file",
    )
    dest_group.add_argument(
        "-R",
        "--replace-in",
        metavar="INREPLACE",
        dest="replace_old",
        type=str,
        help="Filename fragment to replace",
        default="TEST",
    )
    dest_group.add_argument(
        "-O",
        "--replace-out",
        metavar="OUTREPLACE",
        dest="replace_new",
        type=str,
        help="Replacement filename fragment",
        default="TRIM",
    )
    dest_group.add_argument(
        "-D",
        "--out-dir",
        metavar="OUTDIR",
        dest="outdir",
        type=Path,
        help="Different output Directory",
    )
    parser.add_argument(
        "--no-progress",
        dest="progress_bars",
        action="store_false",
        help="Display progressbars",
    )

    args = parser.parse_args()

    shorten_bag(
        *find_files(
            args.infile,
            args.endtime,
            args.outfile,
            (args.replace_old, args.replace_new),
            args.outdir,
        ),
        topic_progressbars=args.progress_bars,
    )
    print("Done")


def find_files(
    infile: Path,
    time,
    outfile: Optional[Path] = None,
    outreplace: Tuple[str, str] = ("TEST", "SHORT"),
    out_dir: Optional[Union[Path, str]] = None,
) -> Union[Tuple[Path, int, Path], List[Tuple[Path, int, Path]]]:
    if infile.is_file() and infile.exists() and (infile.suffix == ".bag"):
        # The bag file is Ok and exitsts.
        if outfile is None:
            # if outreplace[0] in infile.name:
            assert outreplace[0] in infile.name
            assert outreplace[0] != outreplace[1]
            outfile = infile.with_name(infile.name.replace(*outreplace))

        if out_dir is not None:
            ofilename = outfile.name
            if type(out_dir) is str:
                outfile = outfile.parent.joinpath(out_dir, ofilename)
            else:  # Is Path
                outfile = outfile.parent.joinpath(out_dir, ofilename)
            
            if not outfile.parent.exists():
                outfile.parent.mkdir(parents=True)

        return (infile, time, outfile)
    elif infile.is_dir() and infile.exists():
        result: bool = False
        tasks = []
        for file in infile.iterdir():
            if file.is_file() and (file.suffix == ".bag"):
                tasks.append(find_files(file, time, outfile, outreplace, out_dir))
                result = True
        if not result:
            raise FileNotFoundError(f"No bagfiles where found in {infile}")
        else:
            return tasks
    else:
        raise FileNotFoundError("No bagfiles or directory where found")


def shorten_bag(
    infile: Path, endtime: int, outfile: Path, topic_progressbars: bool = True
):
    with Bag(outfile, mode="w") as outbag:
        inbag = Bag(infile)
        starttime = inbag.get_start_time() * 1e9
        duration = endtime - starttime
        last_topic = set()
        if topic_progressbars:
            bars = dict(
                (
                    topic,
                    tqdm.tqdm(
                        desc=topic,
                        total=(int(data.frequency * duration * 1e-9)),
                        unit_scale=True,
                        unit="msgs",
                        position=(index + 1),
                        bar_format="[{percentage:3.1f}%] {desc: <30}[{bar}] {n_fmt: >6}/{total_fmt}|{rate_fmt: >11}",
                    ),
                )
                for (index, (topic, data)) in enumerate(
                    inbag.get_type_and_topic_info()[1].items()
                )
            )

        for topic, msg, timestamp in inbag.read_messages(
            # Do 1 cycle extra
            end_time=bag.genpy.Time.from_sec((endtime + 35000000) * 1e-9)
        ):  # end_time=endtimestamp
            if topic_progressbars:
                bars[topic].update()

            outbag.write(topic, msg, timestamp)
            continue
            if timestamp.nsecs <= endtime or topic not in last_topic:
                if timestamp.nsecs > endtime:
                    last_topic.add(topic)

        for bar in bars.values():
            bar.close()


# shorten_bag(Path("/home/jap/bep/final/bags/BEP-TEST_2023-06-07-12-44-59.bag"), 1686134787193673622, Path("./pfft.bag"))
if __name__ == "__main__":
    main()
