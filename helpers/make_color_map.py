#!/usr/bin/env python3

import argparse
from pathlib import Path
import numpy as np

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Make .cpp source file for a colormap')

    parser.add_argument('--prologue', type=Path, default="prologue.txt",
                        help='Test file containing boilerplate for the beginning of file')
    parser.add_argument('--epilogue', type=Path, default='epilogue.txt',
                        help="Test file containing boilerplate for end of file")

    parser.add_argument('--csv', type=Path, nargs="*",
                        help='CSV file containing')

    args = parser.parse_args()

    if args.prologue:
        with open(args.prologue) as fp:
            for line in fp:
                print(line.strip())

    for csv_file in args.csv:
        data = np.loadtxt(csv_file, delimiter=',', dtype=float)

        print("const float ColorMap::float_data[%d][3] = {" % len(data))
        for entry in data:
            print("{%f,%f,%f}," % (entry[0],entry[1],entry[2]))
        print("};")
        print()

        print("const float ColorMap::char_data[%d][3] = {" % len(data))
        for entry in data:
            print("{%d,%d,%d}," % (int(entry[0]*255),
                                    int(entry[1]*255),
                                    int(entry[2]*255)))
        print("};")
        print()


    if args.epilogue:
        with open(args.epilogue) as fp:
            for line in fp:
                print(line)
